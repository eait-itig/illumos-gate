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
 * COMP3301 prac 4 "sum" device
 *
 * Presents a single BAR which can be used to add numbers together.
 */

#include <sys/cdefs.h>

#include <sys/types.h>

#include <stdio.h>
#include <pthread.h>
#include <stddef.h>

#include "bhyverun.h"
#include "config.h"
#include "debug.h"
#include "pci_emul.h"

struct pci_p4sum_softc {
	struct pci_devinst 	*dsc_pi;
	pthread_mutex_t		 dsc_mtx;
	uint64_t		 dsc_a;
	uint64_t		 dsc_b;
};

struct pci_p4sum_bar {
	uint32_t	 a_lo;
	uint32_t	 a_hi;
	uint32_t	 b_lo;
	uint32_t	 b_hi;
	uint32_t	 sum_lo;
	uint32_t	 sum_hi;
};

static int
pci_p4sum_init(struct vmctx *ctx, struct pci_devinst *pi, nvlist_t *nvl)
{
	struct pci_p4sum_softc *sc;

	sc = calloc(1, sizeof (struct pci_p4sum_softc));
	sc->dsc_pi = pi;
	pthread_mutex_init(&sc->dsc_mtx, NULL);

	pi->pi_arg = sc;

	pci_set_cfgdata16(pi, PCIR_DEVICE, 0x0001);
	pci_set_cfgdata16(pi, PCIR_VENDOR, 0x3301);
	pci_set_cfgdata8(pi, PCIR_CLASS, PCIC_SIMPLECOMM);
	pci_set_cfgdata8(pi, PCIR_SUBCLASS, PCIS_SIMPLECOMM_OTHER);

	pci_emul_add_msicap(pi, 1);

	pci_emul_alloc_bar(pi, 0, PCIBAR_MEM64, sizeof (struct pci_p4sum_bar));

	return (0);
}

static int
pci_p4sum_legacy_config(nvlist_t *nvl, const char *opts)
{
	return (0);
}

static void
pci_p4sum_write(struct vmctx *ctx, int vcpu, struct pci_devinst *pi,
                int baridx, uint64_t offset, int size, uint64_t value)
{
	struct pci_p4sum_softc *sc = pi->pi_arg;

	assert(baridx == 0);
	if (offset >= sizeof (struct pci_p4sum_bar))
		return;

	pthread_mutex_lock(&sc->dsc_mtx);
	switch (offset) {
	case (offsetof(struct pci_p4sum_bar, a_lo)):
		if (size == 8) {
			sc->dsc_a = value;
		} else if (size == 4) {
			sc->dsc_a = (sc->dsc_a & 0xffffffff00000000) | value;
		}
		break;
	case (offsetof(struct pci_p4sum_bar, a_hi)):
		if (size != 4)
			break;
		sc->dsc_a = (sc->dsc_a & 0xffffffff) | (value << 32);
		break;
	case (offsetof(struct pci_p4sum_bar, b_lo)):
		if (size == 8) {
			sc->dsc_b = value;
		} else if (size == 4) {
			sc->dsc_b = (sc->dsc_b & 0xffffffff00000000) | value;
		}
		break;
	case (offsetof(struct pci_p4sum_bar, b_hi)):
		if (size != 4)
			break;
		sc->dsc_b = (sc->dsc_b & 0xffffffff) | (value << 32);
		break;
	}
	pthread_mutex_unlock(&sc->dsc_mtx);
}

static uint64_t
pci_p4sum_read(struct vmctx *ctx, int vcpu, struct pci_devinst *pi, int baridx,
    uint64_t offset, int size)
{
	struct pci_p4sum_softc *sc = pi->pi_arg;
	uint64_t value = 0;

	assert(baridx == 0);

	pthread_mutex_lock(&sc->dsc_mtx);
	switch (offset) {
	case (offsetof(struct pci_p4sum_bar, a_lo)):
		value = sc->dsc_a;
		break;
	case (offsetof(struct pci_p4sum_bar, a_hi)):
		value = sc->dsc_a >> 32;
		break;
	case (offsetof(struct pci_p4sum_bar, b_lo)):
		value = sc->dsc_b;
		break;
	case (offsetof(struct pci_p4sum_bar, b_hi)):
		value = sc->dsc_b >> 32;
		break;
	case (offsetof(struct pci_p4sum_bar, sum_lo)):
		value = sc->dsc_a + sc->dsc_b;
		break;
	case (offsetof(struct pci_p4sum_bar, sum_hi)):
		value = sc->dsc_a + sc->dsc_b;
		value >>= 32;
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

struct pci_devemu pci_de_p4sum = {
	.pe_emu =	"p4sum",
	.pe_init =	pci_p4sum_init,
	.pe_legacy_config = pci_p4sum_legacy_config,
	.pe_barwrite =	pci_p4sum_write,
	.pe_barread =	pci_p4sum_read
};
PCI_EMUL_SET(pci_de_p4sum);
