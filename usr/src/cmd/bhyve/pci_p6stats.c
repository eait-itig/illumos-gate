/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright 2023 The University of Queensland
 * Author: Alex Wilson <alex@uq.edu.au>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * COMP3301 prac 6 "stats" device
 *
 * A simple DMA device for computing statistics about a buffer full of 64-bit
 * integers.
 */

#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/atomic.h>

#include <stdio.h>
#include <pthread.h>
#include <stddef.h>
#include <string.h>

#include "bhyverun.h"
#include "config.h"
#include "debug.h"
#include "pci_emul.h"

struct pci_p6stats_out {
	uint64_t		p6o_count;
	uint64_t		p6o_sum;
	uint64_t		p6o_mean;
	uint64_t		p6o_median;
	uint64_t		p6o_p95;
};

struct pci_p6stats_softc {
	struct pci_devinst 	*dsc_pi;

	pthread_mutex_t		 dsc_mtx;
	pthread_t		 dsc_worker;
	pthread_cond_t		 dsc_work_avail;

	uint64_t		 dsc_inbase;
	uint64_t		*dsc_input;
	size_t			 dsc_incount;
	uint64_t		 dsc_obase;
	struct pci_p6stats_out	*dsc_output;
};

struct pci_p6stats_bar {
	uint32_t		pdb_ibase_lo;
	uint32_t		pdb_ibase_hi;
	uint32_t		pdb_icount_lo;
	uint32_t		pdb_icount_hi;
	uint32_t		pdb_obase_lo;
	uint32_t		pdb_obase_hi;
	uint64_t		pdb_dbell;
};

/*
 *  This Quickselect routine is based on the algorithm described in
 *  "Numerical recipes in C", Second Edition,
 *  Cambridge University Press, 1992, Section 8.5, ISBN 0-521-43108-5
 *  This code by Nicolas Devillard - 1998. Public domain.
 */

#define elem_type	uint64_t
#define ELEM_SWAP(a,b)	\
	do { register elem_type t=(a);(a)=(b);(b)=t; } while (0)

static elem_type
quick_select(elem_type arr[], int n)
{
	int low, high ;
	int median;
	int middle, ll, hh;

	low = 0;
	high = n-1;
	median = (low + high) / 2;
	for (;;) {
		if (high <= low) /* One element only */
			return (arr[median]);

		if (high == low + 1) {  /* Two elements only */
			if (arr[low] > arr[high])
				ELEM_SWAP(arr[low], arr[high]);
			return (arr[median]);
		}

		/* Find median of low, middle and high items; swap into position low */
		middle = (low + high) / 2;
		if (arr[middle] > arr[high])    ELEM_SWAP(arr[middle], arr[high]);
		if (arr[low] > arr[high])       ELEM_SWAP(arr[low], arr[high]);
		if (arr[middle] > arr[low])     ELEM_SWAP(arr[middle], arr[low]);

		/* Swap low item (now in position middle) into position (low+1) */
		ELEM_SWAP(arr[middle], arr[low+1]);

		/* Nibble from each end towards middle, swapping items when stuck */
		ll = low + 1;
		hh = high;
		for (;;) {
			do ll++; while (arr[low] > arr[ll]);
			do hh--; while (arr[hh]  > arr[low]);

			if (hh < ll)
				break;

			ELEM_SWAP(arr[ll], arr[hh]);
		}

		/* Swap middle item (in position low) back into correct position */
		ELEM_SWAP(arr[low], arr[hh]);

		/* Re-set active partition */
		if (hh <= median)
			low = ll;
		if (hh >= median)
			high = hh - 1;
	}
}

#undef ELEM_SWAP
#undef elem_type

static void *
pci_p6stats_worker(void *arg)
{
	struct pci_p6stats_softc *sc = arg;
	uint64_t *input;
	struct pci_p6stats_out *output;
	size_t incount, i;
	uint64_t *tmp = NULL;
	size_t tmpcount = 0;

	pthread_mutex_lock(&sc->dsc_mtx);
	while (1) {
		while (sc->dsc_input == NULL || sc->dsc_output == NULL ||
			sc->dsc_incount == 0) {
			pthread_cond_wait(&sc->dsc_work_avail, &sc->dsc_mtx);
		}
		input = sc->dsc_input;
		output = sc->dsc_output;
		incount = sc->dsc_incount;
		sc->dsc_input = NULL;
		sc->dsc_output = NULL;
		sc->dsc_incount = 0;
		pthread_mutex_unlock(&sc->dsc_mtx);

		membar_consumer();

		if (tmpcount < incount) {
			free(tmp);
			tmp = calloc(incount, sizeof (uint64_t));
			tmpcount = incount;
		}

		bzero(output, sizeof (*output));
		output->p6o_count = incount;
		for (i = 0; i < incount; ++i) {
			output->p6o_sum += input[i];
			tmp[i] = input[i];
		}
		output->p6o_mean = output->p6o_sum / incount;
		output->p6o_median = quick_select(tmp, incount);

		membar_producer();

		pthread_mutex_lock(&sc->dsc_mtx);
		pci_generate_msix(sc->dsc_pi, 0);
	}


	return (NULL);
}

static int
pci_p6stats_init(struct vmctx *ctx, struct pci_devinst *pi, nvlist_t *nvl)
{
	struct pci_p6stats_softc *sc;

	sc = calloc(1, sizeof (struct pci_p6stats_softc));
	sc->dsc_pi = pi;
	pthread_mutex_init(&sc->dsc_mtx, NULL);
	pthread_cond_init(&sc->dsc_work_avail, NULL);

	pi->pi_arg = sc;

	pthread_create(&sc->dsc_worker, NULL, pci_p6stats_worker, sc);
	pthread_setname_np(sc->dsc_worker, "p6stats");

	pci_set_cfgdata16(pi, PCIR_DEVICE, 0x0002);
	pci_set_cfgdata16(pi, PCIR_VENDOR, 0x3301);
	pci_set_cfgdata8(pi, PCIR_CLASS, PCIC_SIMPLECOMM);
	pci_set_cfgdata8(pi, PCIR_SUBCLASS, PCIS_SIMPLECOMM_OTHER);

	pci_emul_alloc_bar(pi, 0, PCIBAR_MEM64,
	    sizeof (struct pci_p6stats_bar));

	pci_emul_add_msixcap(pi, 1, 2);
	pci_emul_add_pciecap(pi, PCIEM_TYPE_ROOT_INT_EP);

	return (0);
}

static int
pci_p6stats_legacy_config(nvlist_t *nvl, const char *opts)
{
	return (0);
}

static void
pci_p6stats_write(struct vmctx *ctx, int vcpu, struct pci_devinst *pi,
    int baridx, uint64_t offset, int size, uint64_t value)
{
	struct pci_p6stats_softc *sc = pi->pi_arg;

	if (baridx == pci_msix_table_bar(pi) ||
	    baridx == pci_msix_pba_bar(pi)) {
		pci_emul_msix_twrite(pi, offset, size, value);
		return;
	}

	assert(baridx == 0);
	if (offset >= sizeof (struct pci_p6stats_bar))
		return;

	pthread_mutex_lock(&sc->dsc_mtx);
	switch (offset) {
	case (offsetof(struct pci_p6stats_bar, pdb_ibase_lo)):
		if (size == 8) {
			sc->dsc_inbase = value;
		} else if (size == 4) {
			sc->dsc_inbase =
			    (sc->dsc_inbase & 0xffffffff00000000) | value;
		} else {
			break;
		}
		sc->dsc_input = paddr_guest2host(pi->pi_vmctx, sc->dsc_inbase,
		    sc->dsc_incount * sizeof(uint64_t));
		break;
	case (offsetof(struct pci_p6stats_bar, pdb_ibase_hi)):
		if (size != 4)
			break;
		sc->dsc_inbase = (sc->dsc_inbase & 0xffffffff) | (value << 32);
		sc->dsc_input = paddr_guest2host(pi->pi_vmctx, sc->dsc_inbase,
		    sc->dsc_incount * sizeof(uint64_t));
		break;
	case (offsetof(struct pci_p6stats_bar, pdb_icount_lo)):
		if (size == 8) {
			sc->dsc_incount = value;
		} else if (size == 4) {
			sc->dsc_incount =
			    (sc->dsc_incount & 0xffffffff00000000) | value;
		} else {
			break;
		}
		sc->dsc_input = paddr_guest2host(pi->pi_vmctx, sc->dsc_inbase,
		    sc->dsc_incount * sizeof(uint64_t));
		break;
	case (offsetof(struct pci_p6stats_bar, pdb_icount_hi)):
		if (size != 4)
			break;
		sc->dsc_incount = (sc->dsc_incount & 0xffffffff) | (value << 32);
		sc->dsc_input = paddr_guest2host(pi->pi_vmctx, sc->dsc_inbase,
		    sc->dsc_incount * sizeof(uint64_t));
		break;
	case (offsetof(struct pci_p6stats_bar, pdb_obase_lo)):
		if (size == 8) {
			sc->dsc_obase = value;
		} else if (size == 4) {
			sc->dsc_obase =
			    (sc->dsc_obase & 0xffffffff00000000) | value;
		} else {
			break;
		}
		sc->dsc_output = paddr_guest2host(pi->pi_vmctx, sc->dsc_obase,
		    sizeof (struct pci_p6stats_out));
		break;
	case (offsetof(struct pci_p6stats_bar, pdb_obase_hi)):
		if (size != 4)
			break;
		sc->dsc_obase = (sc->dsc_obase & 0xffffffff) | (value << 32);
		sc->dsc_output = paddr_guest2host(pi->pi_vmctx, sc->dsc_obase,
		    sizeof (struct pci_p6stats_out));
		break;
	case (offsetof(struct pci_p6stats_bar, pdb_dbell)):
		if (sc->dsc_input != NULL && sc->dsc_output != NULL &&
			sc->dsc_incount != 0) {
			pthread_cond_signal(&sc->dsc_work_avail);
		} else {
			PRINTLN("p6stats: dbell write in invalid state ("
			    "input = %p, output = %p, incount = %llu",
			    sc->dsc_input, sc->dsc_output, sc->dsc_incount);
		}
		break;
	}
	pthread_mutex_unlock(&sc->dsc_mtx);
}

static uint64_t
pci_p6stats_read(struct vmctx *ctx, int vcpu, struct pci_devinst *pi, int baridx,
    uint64_t offset, int size)
{
	struct pci_p6stats_softc *sc = pi->pi_arg;
	uint64_t value = 0;

	if (baridx == pci_msix_table_bar(pi) ||
	    baridx == pci_msix_pba_bar(pi)) {
		return (pci_emul_msix_tread(pi, offset, size));
	}

	assert(baridx == 0);

	pthread_mutex_lock(&sc->dsc_mtx);
	switch (offset) {
	case (offsetof(struct pci_p6stats_bar, pdb_ibase_lo)):
		value = sc->dsc_inbase;
		break;
	case (offsetof(struct pci_p6stats_bar, pdb_ibase_hi)):
		value = sc->dsc_inbase >> 32;
		break;
	case (offsetof(struct pci_p6stats_bar, pdb_icount_lo)):
		value = sc->dsc_incount;
		break;
	case (offsetof(struct pci_p6stats_bar, pdb_icount_hi)):
		value = sc->dsc_incount >> 32;
		break;
	case (offsetof(struct pci_p6stats_bar, pdb_obase_lo)):
		value = sc->dsc_obase;
		break;
	case (offsetof(struct pci_p6stats_bar, pdb_obase_hi)):
		value = sc->dsc_obase >> 32;
		break;
	case (offsetof(struct pci_p6stats_bar, pdb_dbell)):
		value = 0xffffffffffffffff;
		break;
	}
	pthread_mutex_unlock(&sc->dsc_mtx);

	switch (size) {
	case 1:
		value &= 0xFF;
		break;
	case 2:
		value &= 0xFFFF;
		break;
	case 4:
		value &= 0xFFFFFFFF;
		break;
	}

	return (value);
}

struct pci_devemu pci_de_p6stats = {
	.pe_emu =	"p6stats",
	.pe_init =	pci_p6stats_init,
	.pe_legacy_config = pci_p6stats_legacy_config,
	.pe_barwrite =	pci_p6stats_write,
	.pe_barread =	pci_p6stats_read
};
PCI_EMUL_SET(pci_de_p6stats);
