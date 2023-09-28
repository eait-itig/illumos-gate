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
 * COMP3301 assignment 2 device
 */

#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/atomic.h>
#include <sys/uio.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <limits.h>
#include <stdio.h>
#include <pthread.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <port.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>
#include <endian.h>

#include <sys/list.h>

#include "bhyverun.h"
#include "config.h"
#include "debug.h"
#include "pci_emul.h"

#define	A2DEBUG(psc, fmt, arg...)	do { \
		if ((psc)->sc_debug) { \
			fprintf(stderr, "a2: " fmt "\n", ##arg); \
		} \
	} while (0)

#define	PCI_A2_VER_MAJ	0x01
#define	PCI_A2_VER_MIN	0x03

struct pci_a2_bar {
	uint32_t	b_vmaj;
	uint32_t	b_vmin;
	uint32_t	b_flags;
	uint32_t	b_rsvd0;
	uint32_t	b_cbase_lo;
	uint32_t	b_cbase_hi;
	uint32_t	b_cshift;
	uint32_t	b_rsvd1;
	uint32_t	b_rbase_lo;
	uint32_t	b_rbase_hi;
	uint32_t	b_rshift;
	uint32_t	b_rsvd2;
	uint32_t	b_cpbase_lo;
	uint32_t	b_cpbase_hi;
	uint32_t	b_cpshift;
	uint32_t	b_rsvd3;
	uint32_t	b_dbell;
	uint32_t	b_cpdbell;
};

enum a2_descr_owner {
	DEVICE_OWNER 	= 0xAA,
	HOST_OWNER	= 0x55
};

struct pci_a2_descr {
	uint8_t		d_owner;
	uint8_t		d_type;
	uint8_t		d_rsvd0[6];
	uint32_t	d_len[4];
	uint64_t	d_cookie;
	uint64_t	d_ptr[4];
};

struct pci_a2_cdescr {
	uint8_t		cd_owner;
	uint8_t		cd_type;
	uint8_t		cd_rsvd0[6];
	uint32_t	cd_msglen;
	uint32_t	cd_rsvd1;
	uint64_t	cd_cmd_cookie;
	uint64_t	cd_reply_cookie;
};

enum err_flags {
	EFLAG_FLTB	= (1<<0),
	EFLAG_FLTR	= (1<<1),
	EFLAG_DROP	= (1<<2),
	EFLAG_OVF	= (1<<3),
	EFLAG_SEQ	= (1<<4),
	EFLAG_HWERR	= (1<<16),
	EFLAG_RST	= (1<<31)
};

enum conn_state {
	CONN_IDLE 		= 0,	/* none */
	CONN_WRITING,			/* tx */
	CONN_WAITBUFS,			/* none */
	CONN_READHDR,			/* rx[0] only (c_head) */
	CONN_READING,			/* rx */
	CONN_RECONNECT,
};

enum conn_evt {
	EVT_ENTRY,
	EVT_WRITABLE,
	EVT_READABLE,
	EVT_TX_BUFS,
	EVT_RX_BUFS,
};

enum a2_err_cmd {
	CMD_A2_HWERR_WRITE	= 0xf0,
	CMD_A2_HWERR_READ	= 0xf1,
	CMD_A2_DROP		= 0xf2
};

struct pci_a2_softc {
	struct pci_devinst	*sc_pi;
	pthread_mutex_t		 sc_mtx;

	char			 sc_sockpath[PATH_MAX];

	int			 sc_txport;
	int			 sc_ioport;

	int			 sc_debug;

	uint32_t		 sc_eflags;

	uint			 sc_workers_started;
	pthread_t		 sc_io_worker;
	pthread_t		 sc_tx_worker;

	uint64_t		 sc_cbase;
	uint32_t		 sc_cshift;
	uint64_t		 sc_rbase;
	uint32_t		 sc_rshift;
	uint64_t		 sc_cpbase;
	uint32_t		 sc_cpshift;
	uint32_t		 sc_last_cdbell;
	uint32_t		 sc_last_rdbell;
	uint32_t		 sc_last_cpdbell;

	struct pci_a2_descr	*sc_cmdr;
	size_t			 sc_maxcmd;
	struct pci_a2_descr	*sc_replyr;
	size_t			 sc_maxreply;
	struct pci_a2_cdescr	*sc_compr;
	size_t			 sc_maxcomp;

	list_t			 sc_c_all;
	list_t			 sc_c_tx_bufs;
	list_t			 sc_c_rx_bufs;

	list_t			 sc_txcs;
	list_t			 sc_rxcs;

	int			 sc_intr;
	uint			 sc_cpc;	/* comp ring prod counter */
};

enum a2_port_events {
	PORTEV_A2_WAKEUP 	= 0x3301,
	PORTEV_A2_STOP,
};

struct pci_a2_seg {
	char			*s_base;
	size_t			 s_len;
	size_t			 s_pos;
};

struct pci_a2_head {
	uint32_t		 h_len;
	uint8_t			 h_type;
} __packed;

struct pci_a2_conn {
	pthread_mutex_t		 c_mtx;
	enum conn_state		 c_state;
	struct pci_a2_softc	*c_sc;
	int			 c_fd;
	int			 c_closing;
	list_node_t		 c_all_node;
	list_node_t		 c_tx_bufs_node;
	list_node_t		 c_rx_bufs_node;
	struct pci_a2_descr	*c_cmd_descr;
	struct pci_a2_descr	*c_reply_descr;
	uint64_t		 c_cmd_cookie;
	uint64_t		 c_reply_cookie;
	struct pci_a2_head	 c_head;
	struct pci_a2_seg	 c_tx_segs[5];
	struct pci_a2_seg	 c_rx_segs[5];
};

/* max events per port_getn call */
#define	EVN		8
/* number of uds connections to keep open */
#define	SOCKCONNS	16

static void
pci_a2_teardown_conns(struct pci_a2_softc *sc)
{
	struct pci_a2_conn *c;

	while ((c = list_remove_head(&sc->sc_txcs)) != NULL) {
		list_remove(&sc->sc_c_all, c);
		pthread_mutex_destroy(&c->c_mtx);
		close(c->c_fd);
		free(c);
	}
	while ((c = list_remove_head(&sc->sc_rxcs)) != NULL) {
		list_remove(&sc->sc_c_all, c);
		pthread_mutex_destroy(&c->c_mtx);
		close(c->c_fd);
		free(c);
	}
	assert(list_is_empty(&sc->sc_txcs));
	assert(list_is_empty(&sc->sc_rxcs));

	while ((c = list_remove_head(&sc->sc_c_all)) != NULL) {
		if (list_link_active(&c->c_tx_bufs_node))
			list_remove(&sc->sc_c_tx_bufs, c);
		if (list_link_active(&c->c_rx_bufs_node))
			list_remove(&sc->sc_c_rx_bufs, c);
		pthread_mutex_destroy(&c->c_mtx);
		close(c->c_fd);
		free(c);
	}
	assert(list_is_empty(&sc->sc_c_all));
	assert(list_is_empty(&sc->sc_c_tx_bufs));
	assert(list_is_empty(&sc->sc_c_rx_bufs));
}

static void
pci_a2_error_locked(struct pci_a2_softc *sc, enum err_flags errflag)
{
	pthread_t self = pthread_self();

	sc->sc_eflags = errflag;
	sc->sc_workers_started = 0;

	if (pthread_equal(self, sc->sc_io_worker)) {
		port_send(sc->sc_txport, PORTEV_A2_STOP, NULL);
		pthread_mutex_unlock(&sc->sc_mtx);
		pthread_join(sc->sc_tx_worker, NULL);

		pthread_mutex_lock(&sc->sc_mtx);
		pci_a2_teardown_conns(sc);
		close(sc->sc_ioport);
		sc->sc_ioport = -1;

		pci_generate_msix(sc->sc_pi, 1);

		pthread_mutex_unlock(&sc->sc_mtx);
		pthread_exit(NULL);

	} else if (pthread_equal(self, sc->sc_tx_worker)) {
		close(sc->sc_txport);
		sc->sc_txport = -1;

		port_send(sc->sc_ioport, PORTEV_A2_STOP, NULL);
		pthread_mutex_unlock(&sc->sc_mtx);
		pthread_join(sc->sc_io_worker, NULL);

		pthread_mutex_lock(&sc->sc_mtx);
		pci_generate_msix(sc->sc_pi, 1);
		pthread_mutex_unlock(&sc->sc_mtx);

		pthread_exit(NULL);

	} else {
		port_send(sc->sc_ioport, PORTEV_A2_STOP, NULL);
		port_send(sc->sc_txport, PORTEV_A2_STOP, NULL);
		pthread_mutex_unlock(&sc->sc_mtx);
		pthread_join(sc->sc_tx_worker, NULL);
		pthread_join(sc->sc_io_worker, NULL);

		pthread_mutex_lock(&sc->sc_mtx);
		pci_generate_msix(sc->sc_pi, 1);
		pthread_mutex_unlock(&sc->sc_mtx);
	}
}

static void
pci_a2_error(struct pci_a2_softc *sc, enum err_flags errflag)
{
	pthread_mutex_lock(&sc->sc_mtx);
	pci_a2_error_locked(sc, errflag);
}

static struct pci_a2_conn *
pci_a2_new_conn(struct pci_a2_softc *sc)
{
	struct sockaddr_un un;
	int fd;
	struct pci_a2_conn *c;

	fd = socket(PF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);
	if (fd < 0)
		return (NULL);

	bzero(&un, sizeof (un));
	un.sun_family = AF_UNIX;
	strlcpy(un.sun_path, sc->sc_sockpath, sizeof (un.sun_path));
	if (connect(fd, (const struct sockaddr *)&un, sizeof (un))) {
		A2DEBUG(sc, "connect fail: %d: %s", errno, strerror(errno));
		close(fd);
		return (NULL);
	}

	c = calloc(1, sizeof (*c));
	c->c_fd = fd;
	c->c_sc = sc;

	pthread_mutex_init(&c->c_mtx, NULL);

	pthread_mutex_lock(&sc->sc_mtx);
	list_insert_tail(&sc->sc_c_all, c);
	pthread_mutex_unlock(&sc->sc_mtx);

	return (c);
}

typedef enum {
	RW_DONE,
	RW_EOF,
	RW_AGAIN,
	RW_ERROR
} conn_rw_ret_t;

static conn_rw_ret_t
pci_a2_conn_write(struct pci_a2_conn *c)
{
	struct iovec iov[5];
	uint cnt = 0;
	uint i;
	struct pci_a2_seg *seg;
	ssize_t done;
	size_t take, sent = 0;
	struct msghdr hdr;

	bzero(iov, sizeof (iov));

	for (i = 0; i < 5; ++i) {
		seg = &c->c_tx_segs[i];
		if (seg->s_pos >= seg->s_len)
			continue;
		iov[cnt].iov_base = seg->s_base + seg->s_pos;
		iov[cnt].iov_len = seg->s_len - seg->s_pos;
		sent += iov[cnt].iov_len;
		++cnt;
	}

	if (cnt == 0)
		return (RW_DONE);

	bzero(&hdr, sizeof (hdr));
	hdr.msg_iov = iov;
	hdr.msg_iovlen = cnt;
	done = sendmsg(c->c_fd, &hdr, MSG_DONTWAIT | MSG_NOSIGNAL);
	if (done < 0) {
		switch (errno) {
		case EAGAIN:
#if defined(EWOULDBLOCK) && EWOULDBLOCK != EAGAIN
		case EWOULDBLOCK:
#endif
			return (RW_AGAIN);
		}
		return (RW_ERROR);
	}
	sent -= done;

	for (i = 0; i < 5 && done > 0; ++i) {
		seg = &c->c_tx_segs[i];
		if (seg->s_pos >= seg->s_len)
			continue;
		take = seg->s_len - seg->s_pos;
		if (take > done)
			take = done;
		seg->s_pos += take;
		done -= take;
	}

	if (sent > 0)
		return (RW_AGAIN);

	return (RW_DONE);
}

static conn_rw_ret_t
pci_a2_conn_read(struct pci_a2_conn *c)
{
	struct iovec iov[5];
	uint cnt = 0;
	uint i;
	struct pci_a2_seg *seg;
	ssize_t done;
	size_t take, sent = 0;

	bzero(iov, sizeof (iov));

	for (i = 0; i < 5; ++i) {
		seg = &c->c_rx_segs[i];
		if (seg->s_pos >= seg->s_len)
			continue;
		iov[cnt].iov_base = seg->s_base + seg->s_pos;
		iov[cnt].iov_len = seg->s_len - seg->s_pos;
		sent += iov[cnt].iov_len;
		++cnt;
	}

	if (cnt == 0)
		return (RW_DONE);

	done = readv(c->c_fd, iov, cnt);
	if (done < 0) {
		switch (errno) {
		case EAGAIN:
#if defined(EWOULDBLOCK) && EWOULDBLOCK != EAGAIN
		case EWOULDBLOCK:
#endif
			return (RW_AGAIN);
		}
		return (RW_ERROR);
	} else if (done == 0) {
		return (RW_EOF);
	}
	sent -= done;

	for (i = 0; i < 5 && done > 0; ++i) {
		seg = &c->c_rx_segs[i];
		if (seg->s_pos >= seg->s_len)
			continue;
		take = seg->s_len - seg->s_pos;
		if (take > done)
			take = done;
		seg->s_pos += take;
		done -= take;
	}

	if (sent > 0)
		return (RW_AGAIN);

	return (RW_DONE);
}

static void
pci_a2_setup_rx_hdr_buf(struct pci_a2_conn *c)
{
	bzero(&c->c_head, sizeof (c->c_head));
	bzero(c->c_rx_segs, sizeof (c->c_rx_segs));
	c->c_rx_segs[0].s_base = (char *)&c->c_head;
	c->c_rx_segs[0].s_len = sizeof (c->c_head);

	A2DEBUG(c->c_sc,
	    "conn %p: rx segs for hdr: %p+%x, %p+%x, %p+%x, %p+%x, %p+%x",
	    c, c->c_rx_segs[0].s_base, c->c_rx_segs[0].s_len,
	    c->c_rx_segs[1].s_base, c->c_rx_segs[1].s_len,
	    c->c_rx_segs[2].s_base, c->c_rx_segs[2].s_len,
	    c->c_rx_segs[3].s_base, c->c_rx_segs[3].s_len,
	    c->c_rx_segs[4].s_base, c->c_rx_segs[4].s_len);
}

static void
pci_a2_rx_trunc_to_hdr(struct pci_a2_conn *c)
{
	size_t want = be32toh(c->c_head.h_len) - 1;
	uint i;
	struct pci_a2_seg *seg;

	for (i = 0; i < 5; ++i) {
		seg = &c->c_rx_segs[i];
		if (seg->s_len == 0)
			continue;
		if (seg->s_len > want)
			seg->s_len = want;
		want -= seg->s_len;
	}

	/*
	 * If we're truncating the received data, then we should close the
	 * socket afterwards to avoid losing sync.
	 */
	if (want > 0)
		c->c_closing = 1;

	A2DEBUG(c->c_sc,
	    "conn %p: rx segs post-trunc: %p+%x, %p+%x, %p+%x, %p+%x, %p+%x",
	    c, c->c_rx_segs[0].s_base, c->c_rx_segs[0].s_len,
	    c->c_rx_segs[1].s_base, c->c_rx_segs[1].s_len,
	    c->c_rx_segs[2].s_base, c->c_rx_segs[2].s_len,
	    c->c_rx_segs[3].s_base, c->c_rx_segs[3].s_len,
	    c->c_rx_segs[4].s_base, c->c_rx_segs[4].s_len);
}

static void
pci_a2_cmd_complete(struct pci_a2_conn *c)
{
	struct pci_a2_softc *sc = c->c_sc;
	struct pci_a2_cdescr *cd;

	pthread_mutex_lock(&sc->sc_mtx);
	if (sc->sc_compr == NULL) {
		A2DEBUG(sc, "compr null at cmd_complete");
		pci_a2_error_locked(sc, EFLAG_FLTB);
		return;
	}
	if (sc->sc_cpc == sc->sc_last_cpdbell) {
		A2DEBUG(sc, "cpc = last_cpdbell, overflow");
		pci_a2_error_locked(sc, EFLAG_OVF);
		return;
	}
	cd = &sc->sc_compr[sc->sc_cpc];
	A2DEBUG(sc, "writing cmd completion for conn %p in slot %d", c,
	    sc->sc_cpc);
	if (++sc->sc_cpc >= sc->sc_maxcomp)
		sc->sc_cpc = 0;
	pthread_mutex_unlock(&sc->sc_mtx);

	cd->cd_type = 0;
	cd->cd_msglen = 0;
	cd->cd_cmd_cookie = c->c_cmd_cookie;
	cd->cd_reply_cookie = 0;
	membar_producer();
	if (cd->cd_owner != DEVICE_OWNER) {
		A2DEBUG(sc, "comp desc not DEVICE_OWNER");
		pci_a2_error(sc, EFLAG_OVF);
		return;
	}
	cd->cd_owner = HOST_OWNER;

	pthread_mutex_lock(&sc->sc_mtx);
	sc->sc_intr = 1;
	pthread_mutex_unlock(&sc->sc_mtx);
}

static void
pci_a2_reply_complete(struct pci_a2_conn *c)
{
	struct pci_a2_softc *sc = c->c_sc;
	struct pci_a2_cdescr *cd;

	pthread_mutex_lock(&sc->sc_mtx);
	if (sc->sc_compr == NULL) {
		A2DEBUG(sc, "compr null at reply_complete");
		pci_a2_error_locked(sc, EFLAG_FLTB);
		return;
	}
	if (sc->sc_cpc == sc->sc_last_cpdbell) {
		A2DEBUG(sc, "cpc = last_cpdbell, overflow");
		pci_a2_error_locked(sc, EFLAG_OVF);
		return;
	}
	cd = &sc->sc_compr[sc->sc_cpc];
	if (cd->cd_owner != DEVICE_OWNER) {
		A2DEBUG(sc, "cd_owner != DEVICE_OWNER, overflow");
		pci_a2_error_locked(sc, EFLAG_OVF);
		return;
	}
	A2DEBUG(sc, "writing reply completion for conn %p in slot %d", c,
	    sc->sc_cpc);
	if (++sc->sc_cpc >= sc->sc_maxcomp)
		sc->sc_cpc = 0;
	pthread_mutex_unlock(&sc->sc_mtx);

	cd->cd_type = c->c_head.h_type;
	cd->cd_msglen = be32toh(c->c_head.h_len) - 1;
	cd->cd_cmd_cookie = c->c_cmd_cookie;
	cd->cd_reply_cookie = c->c_reply_cookie;
	membar_producer();
	cd->cd_owner = HOST_OWNER;

	pthread_mutex_lock(&sc->sc_mtx);
	sc->sc_intr = 1;
	pthread_mutex_unlock(&sc->sc_mtx);
}

static void
pci_a2_setup_tx_bufs(struct pci_a2_conn *c, struct pci_a2_descr *d)
{
	struct pci_a2_softc *sc = c->c_sc;
	size_t len;
	uint i;

	bzero(c->c_tx_segs, sizeof (c->c_tx_segs));

	c->c_tx_segs[0].s_base = (char *)&c->c_head;
	c->c_tx_segs[0].s_len = sizeof (c->c_head);

	len = 0;
	for (i = 0; i < 4; ++i) {
		if (d->d_len[i] == 0)
			break;
		c->c_tx_segs[i + 1].s_base = paddr_guest2host(
		    sc->sc_pi->pi_vmctx, d->d_ptr[i], d->d_len[i]);
		if (c->c_tx_segs[i + 1].s_base == NULL) {
			A2DEBUG(sc, "bad pointer in descr: %p", d->d_ptr[i]);
			pci_a2_error(sc, EFLAG_FLTR);
			return;
		}
		c->c_tx_segs[i + 1].s_len = d->d_len[i];
		len += d->d_len[i];
	}
	A2DEBUG(sc, "conn %p: tx segs: %p+%x, %p+%x, %p+%x, %p+%x, %p+%x",
	    c, c->c_tx_segs[0].s_base, c->c_tx_segs[0].s_len,
	    c->c_tx_segs[1].s_base, c->c_tx_segs[1].s_len,
	    c->c_tx_segs[2].s_base, c->c_tx_segs[2].s_len,
	    c->c_tx_segs[3].s_base, c->c_tx_segs[3].s_len,
	    c->c_tx_segs[4].s_base, c->c_tx_segs[4].s_len);

	c->c_head.h_type = d->d_type;
	c->c_head.h_len = htobe32(len + 1);
}

static void
pci_a2_setup_rx_bufs(struct pci_a2_conn *c, struct pci_a2_descr *d)
{
	uint i;
	struct pci_a2_softc *sc = c->c_sc;

	bzero(c->c_rx_segs, sizeof (c->c_rx_segs));
	for (i = 0; i < 4; ++i) {
		if (d->d_len[i] == 0)
			break;
		c->c_rx_segs[i].s_base = paddr_guest2host(
		    sc->sc_pi->pi_vmctx, d->d_ptr[i], d->d_len[i]);
		if (c->c_rx_segs[i].s_base == NULL) {
			A2DEBUG(sc, "bad ptr in reply descr: %p+%x",
			    d->d_ptr[i], d->d_len[i]);
			pci_a2_error(sc, EFLAG_FLTR);
			return;
		}
		c->c_rx_segs[i].s_len = d->d_len[i];
	}
	A2DEBUG(sc, "conn %p: rx segs: %p+%x, %p+%x, %p+%x, %p+%x, %p+%x",
	    c, c->c_rx_segs[0].s_base, c->c_rx_segs[0].s_len,
	    c->c_rx_segs[1].s_base, c->c_rx_segs[1].s_len,
	    c->c_rx_segs[2].s_base, c->c_rx_segs[2].s_len,
	    c->c_rx_segs[3].s_base, c->c_rx_segs[3].s_len,
	    c->c_rx_segs[4].s_base, c->c_rx_segs[4].s_len);
}

static inline const char *
conn_state_str(enum conn_state st)
{
	switch (st) {
	case CONN_IDLE: return ("IDLE");
	case CONN_WRITING: return ("WRITING");
	case CONN_WAITBUFS: return ("WAITBUFS");
	case CONN_READHDR: return ("READHDR");
	case CONN_READING: return ("READING");
	case CONN_RECONNECT: return ("RECONNECT");
	default: return ("???");
	}
}

static inline const char *
conn_evt_str(enum conn_evt evt)
{
	switch (evt) {
	case EVT_ENTRY: return ("ENTRY");
	case EVT_WRITABLE: return ("WRITABLE");
	case EVT_READABLE: return ("READABLE");
	case EVT_TX_BUFS: return ("TX_BUFS");
	case EVT_RX_BUFS: return ("RX_BUFS");
	default: return ("???");
	}
}

static void
pci_a2_conn_fsm(struct pci_a2_conn *c, enum conn_evt evt)
{
	struct pci_a2_softc *sc = c->c_sc;
	enum conn_state newstate = c->c_state;
	struct pci_a2_conn *nc;

	A2DEBUG(sc, "conn %p: event %d (%s) in state %d (%s)", c, evt,
	    conn_evt_str(evt), c->c_state, conn_state_str(c->c_state));

	switch (c->c_state) {
	case CONN_IDLE:
		switch (evt) {
		case EVT_ENTRY:
			assert(!list_link_active(&c->c_tx_bufs_node));
			assert(!list_link_active(&c->c_rx_bufs_node));
			pthread_mutex_lock(&sc->sc_mtx);
			list_insert_tail(&sc->sc_c_tx_bufs, c);
			pthread_mutex_unlock(&sc->sc_mtx);
			break;
		case EVT_TX_BUFS:
			c->c_cmd_cookie = c->c_cmd_descr->d_cookie;
			if (c->c_cmd_descr->d_type == CMD_A2_HWERR_WRITE)
				pci_a2_error(sc, EFLAG_HWERR);
			newstate = CONN_WRITING;
			break;
		default:
			assert(0);
		}
		break;
	case CONN_WRITING:
		switch (evt) {
		case EVT_ENTRY:
			pci_a2_setup_tx_bufs(c, c->c_cmd_descr);
			/* FALLTHROUGH */
		case EVT_WRITABLE:
			switch (pci_a2_conn_write(c)) {
			case RW_AGAIN:
				port_associate(sc->sc_ioport, PORT_SOURCE_FD,
				    c->c_fd, POLLOUT, c);
				break;
			case RW_EOF:
			case RW_ERROR:
				A2DEBUG(sc, "i/o error while writing");
				pci_a2_error(sc, EFLAG_HWERR);
				break;
			case RW_DONE:
				pci_a2_cmd_complete(c);
				newstate = CONN_READHDR;
				break;
			}
			break;
		default:
			assert(0);
		}
		break;
	case CONN_WAITBUFS:
		switch (evt) {
		case EVT_ENTRY:
			if (c->c_cmd_descr->d_type == CMD_A2_DROP)
				pci_a2_error(sc, EFLAG_DROP);
			pthread_mutex_lock(&sc->sc_mtx);
			list_insert_tail(&sc->sc_c_rx_bufs, c);
			pthread_mutex_unlock(&sc->sc_mtx);
			break;
		case EVT_RX_BUFS:
			c->c_reply_cookie = c->c_reply_descr->d_cookie;
			newstate = CONN_READING;
			break;
		default:
			assert(0);
		}
		break;
	case CONN_READHDR:
		switch (evt) {
		case EVT_ENTRY:
			pci_a2_setup_rx_hdr_buf(c);
			/* FALLTHROUGH */
		case EVT_READABLE:
			if (c->c_cmd_descr->d_type == CMD_A2_HWERR_READ)
				pci_a2_error(sc, EFLAG_HWERR);
			switch (pci_a2_conn_read(c)) {
			case RW_AGAIN:
				port_associate(sc->sc_ioport, PORT_SOURCE_FD,
				    c->c_fd, POLLIN, c);
				break;
			case RW_ERROR:
			case RW_EOF:
				A2DEBUG(sc, "i/o error while reading hdr");
				pci_a2_error(sc, EFLAG_HWERR);
				break;
			case RW_DONE:
				newstate = CONN_WAITBUFS;
				break;
			}
			break;
		default:
			assert(0);
		}
		break;
	case CONN_READING:
		switch (evt) {
		case EVT_ENTRY:
			if (be32toh(c->c_head.h_len) <= 1) {
				pci_a2_reply_complete(c);
				pthread_mutex_lock(&sc->sc_mtx);
				sc->sc_intr = 1;
				pthread_mutex_unlock(&sc->sc_mtx);
				newstate = CONN_IDLE;
				break;
			}
			pci_a2_setup_rx_bufs(c, c->c_reply_descr);
			pci_a2_rx_trunc_to_hdr(c);
			/* FALLTHROUGH */
		case EVT_READABLE:
			switch (pci_a2_conn_read(c)) {
			case RW_AGAIN:
				port_associate(sc->sc_ioport, PORT_SOURCE_FD,
				    c->c_fd, POLLIN, c);
				break;
			case RW_ERROR:
			case RW_EOF:
				A2DEBUG(sc, "i/o error while reading data");
				pci_a2_error(sc, EFLAG_HWERR);
				break;
			case RW_DONE:
				pci_a2_reply_complete(c);
				pthread_mutex_lock(&sc->sc_mtx);
				sc->sc_intr = 1;
				pthread_mutex_unlock(&sc->sc_mtx);
				if (c->c_closing)
					newstate = CONN_RECONNECT;
				else
					newstate = CONN_IDLE;
				break;
			}
			break;
		default:
			assert(0);
		}
		break;
	case CONN_RECONNECT:
		switch (evt) {
		case EVT_ENTRY:
			pthread_mutex_lock(&sc->sc_mtx);
			assert(!list_link_active(&c->c_tx_bufs_node));
			assert(!list_link_active(&c->c_rx_bufs_node));
			list_remove(&sc->sc_c_all, c);
			pthread_mutex_unlock(&sc->sc_mtx);

			close(c->c_fd);
			free(c);
			nc = pci_a2_new_conn(sc);
			if (nc != NULL)
				pci_a2_conn_fsm(nc, EVT_ENTRY);
			return;
		default:
			assert(0);
		}
		break;
	}
	A2DEBUG(sc, "conn %p: out state %d (%s)", c, newstate,
	    conn_state_str(newstate));
	if (newstate != c->c_state) {
		c->c_state = newstate;
		pci_a2_conn_fsm(c, EVT_ENTRY);
	}
}

static void *
pci_a2_tx_worker(void *arg)
{
	struct pci_a2_softc *sc = arg;
	port_event_t pe;
	uint32_t cmdcc = 0;
	struct pci_a2_descr *d, *cmdr;
	size_t maxcmd;
	struct pci_a2_conn *c;
	int rc;
	uint_t i;
	struct timespec timeout;

	while (1) {
		bzero(&timeout, sizeof (timeout));
		timeout.tv_nsec = 2500000;
		rc = port_get(sc->sc_txport, &pe, &timeout);
		if (rc < 0 && errno != ETIME) {
			A2DEBUG(sc, "port_get failed: %d (%s)", rc,
			    strerror(rc));
			break;
		}
		if (rc == 0) {
			if (pe.portev_source == PORT_SOURCE_USER &&
			    pe.portev_events == PORTEV_A2_STOP) {
				A2DEBUG(sc, "tx worker stop");
				break;
			} else if (pe.portev_source == PORT_SOURCE_USER &&
			    pe.portev_events == PORTEV_A2_WAKEUP) {
				A2DEBUG(sc, "tx worker wakeup!");
			} else {
				assert(0);
			}
		}

		pthread_mutex_lock(&sc->sc_mtx);
		cmdr = sc->sc_cmdr;
		maxcmd = sc->sc_maxcmd;

		if (maxcmd == 0) {
			pthread_mutex_unlock(&sc->sc_mtx);
			continue;
		}

		if (cmdr == NULL) {
			A2DEBUG(sc, "cmdr null but maxcmd set");
			pci_a2_error_locked(sc, EFLAG_FLTB);
			break;
		}

		if (list_is_empty(&sc->sc_c_all)) {
			pthread_mutex_unlock(&sc->sc_mtx);
			A2DEBUG(sc, "worker making conns");
			for (i = 0; i < SOCKCONNS; ++i) {
				c = pci_a2_new_conn(sc);
				if (c == NULL) {
					A2DEBUG(sc, "error connecting");
					pci_a2_error(sc, EFLAG_HWERR);
				}
				pthread_mutex_lock(&c->c_mtx);
				pci_a2_conn_fsm(c, EVT_ENTRY);
				pthread_mutex_unlock(&c->c_mtx);
			}
			pthread_mutex_lock(&sc->sc_mtx);
		}

		list_move_tail(&sc->sc_txcs, &sc->sc_c_tx_bufs);
		pthread_mutex_unlock(&sc->sc_mtx);

		while ((c = list_remove_head(&sc->sc_txcs)) != NULL) {
			d = &cmdr[cmdcc];
			for (i = 0; i < (1<<10); ++i) {
				if (d->d_owner == DEVICE_OWNER)
					break;
				membar_consumer();
			}
			if (d->d_owner == DEVICE_OWNER) {
				membar_consumer();
				pthread_mutex_lock(&c->c_mtx);
				c->c_cmd_descr = d;
				d->d_owner = HOST_OWNER;
				pci_a2_conn_fsm(c, EVT_TX_BUFS);
				pthread_mutex_unlock(&c->c_mtx);
				if (++cmdcc >= maxcmd)
					cmdcc = 0;
				d = &cmdr[cmdcc];
			} else {
				list_insert_tail(&sc->sc_txcs, c);
				break;
			}
		}

		if (!list_is_empty(&sc->sc_txcs)) {
			pthread_mutex_lock(&sc->sc_mtx);
			list_move_tail(&sc->sc_c_tx_bufs, &sc->sc_txcs);
			pthread_mutex_unlock(&sc->sc_mtx);
		}
	}

	pthread_mutex_lock(&sc->sc_mtx);
	close(sc->sc_txport);
	sc->sc_txport = -1;
	pthread_mutex_unlock(&sc->sc_mtx);

	return (NULL);
}

static void *
pci_a2_io_worker(void *arg)
{
	struct pci_a2_softc *sc = arg;
	port_event_t pe[EVN];
	uint_t n, i;
	int rc;
	uint32_t repcc = 0;
	struct pci_a2_descr *d, *replyr;
	size_t maxreply;
	struct pci_a2_conn *c;
	struct timespec timeout;

	while (1) {
		n = EVN;
		bzero(&timeout, sizeof (timeout));
		timeout.tv_nsec = 250000;
		rc = port_getn(sc->sc_ioport, pe, EVN, &n, &timeout);
		if (rc < 0 && errno != ETIME) {
			A2DEBUG(sc, "port_getn failed: %d (%s)", rc,
			    strerror(rc));
			return (NULL);
		}
		if (n > 0)
			A2DEBUG(sc, "worker phase 0, n = %d", n);
		for (i = 0; i < n; ++i) {
			switch (pe[i].portev_source) {
			case PORT_SOURCE_USER:
				switch (pe[i].portev_events) {
				case PORTEV_A2_WAKEUP:
					A2DEBUG(sc, "worker wakeup!");
					break;
				case PORTEV_A2_STOP:
					A2DEBUG(sc, "worker stop");
					goto out;
				default:
					assert(0);
				}
				break;
			case PORT_SOURCE_FD:
				A2DEBUG(sc, "worker fd event on %d",
				    pe[i].portev_source);
				c = pe[i].portev_user;
				pthread_mutex_lock(&c->c_mtx);
				if (pe[i].portev_events & POLLIN)
					pci_a2_conn_fsm(c, EVT_READABLE);
				else if (pe[i].portev_events & POLLOUT)
					pci_a2_conn_fsm(c, EVT_WRITABLE);
				pthread_mutex_unlock(&c->c_mtx);
				break;
			default:
				assert(0);
			}
		}

		pthread_mutex_lock(&sc->sc_mtx);
		replyr = sc->sc_replyr;
		maxreply = sc->sc_maxreply;

		if (maxreply == 0) {
			pthread_mutex_unlock(&sc->sc_mtx);
			continue;
		}
		if (replyr == NULL) {
			A2DEBUG(sc, "replyr null but maxreply set");
			pci_a2_error_locked(sc, EFLAG_FLTB);
			return (NULL);
		}

		list_move_tail(&sc->sc_rxcs, &sc->sc_c_rx_bufs);
		pthread_mutex_unlock(&sc->sc_mtx);

		while ((c = list_remove_head(&sc->sc_rxcs)) != NULL) {
			d = &replyr[repcc];
			for (i = 0; i < 16; ++i) {
				if (d->d_owner == DEVICE_OWNER)
					break;
				membar_consumer();
			}
			if (d->d_owner == DEVICE_OWNER) {
				membar_consumer();
				pthread_mutex_lock(&c->c_mtx);
				c->c_reply_descr = d;
				d->d_owner = HOST_OWNER;
				pci_a2_conn_fsm(c, EVT_RX_BUFS);
				pthread_mutex_unlock(&c->c_mtx);
				if (++repcc >= maxreply)
					repcc = 0;
				d = &replyr[repcc];
			} else {
				A2DEBUG(sc, "at repcc = %u, still HOST_OWNER "
				    "but we need a buf for %p", repcc, c);
				pci_a2_error(sc, EFLAG_DROP);
			}
		}
		assert(list_is_empty(&sc->sc_rxcs));

		pthread_mutex_lock(&sc->sc_mtx);
		if (sc->sc_intr) {
			sc->sc_intr = 0;
			pci_generate_msix(sc->sc_pi, 0);
		}
		pthread_mutex_unlock(&sc->sc_mtx);
	}

out:
	pthread_mutex_lock(&sc->sc_mtx);
	pci_a2_teardown_conns(sc);
	close(sc->sc_ioport);
	sc->sc_ioport = -1;
	pthread_mutex_unlock(&sc->sc_mtx);
	return (NULL);
}

static int
pci_a2_init(struct vmctx *ctx, struct pci_devinst *pi, nvlist_t *nvl)
{
	struct pci_a2_softc *sc;
	const char *value;

	value = get_config_value_node(nvl, "socket");
	if (value == NULL)
		return (-1);

	sc = calloc(1, sizeof (struct pci_a2_softc));
	strlcpy(sc->sc_sockpath, value, sizeof (sc->sc_sockpath));
	sc->sc_pi = pi;
	sc->sc_ioport = port_create();
	sc->sc_txport = port_create();
	pthread_mutex_init(&sc->sc_mtx, NULL);

	value = get_config_value_node(nvl, "debug");
	sc->sc_debug = (value != NULL);

	pi->pi_arg = sc;

	sc->sc_last_rdbell = 0xffffffff;
	sc->sc_last_cdbell = 0xffffffff;
	sc->sc_last_cpdbell = 0xffffffff;

	list_create(&sc->sc_c_all, sizeof (struct pci_a2_conn),
	    offsetof(struct pci_a2_conn, c_all_node));
	list_create(&sc->sc_c_tx_bufs, sizeof (struct pci_a2_conn),
	    offsetof(struct pci_a2_conn, c_tx_bufs_node));
	list_create(&sc->sc_c_rx_bufs, sizeof (struct pci_a2_conn),
	    offsetof(struct pci_a2_conn, c_rx_bufs_node));

	list_create(&sc->sc_txcs, sizeof (struct pci_a2_conn),
	    offsetof(struct pci_a2_conn, c_tx_bufs_node));
	list_create(&sc->sc_rxcs, sizeof (struct pci_a2_conn),
	    offsetof(struct pci_a2_conn, c_rx_bufs_node));

	pthread_create(&sc->sc_io_worker, NULL, pci_a2_io_worker, sc);
	pthread_setname_np(sc->sc_io_worker, "a2-io");
	pthread_create(&sc->sc_tx_worker, NULL, pci_a2_tx_worker, sc);
	pthread_setname_np(sc->sc_tx_worker, "a2-tx");
	sc->sc_workers_started = 1;

	pci_set_cfgdata16(pi, PCIR_DEVICE, 0x0200);
	pci_set_cfgdata16(pi, PCIR_VENDOR, 0x3301);
	pci_set_cfgdata8(pi, PCIR_CLASS, PCIC_SIMPLECOMM);
	pci_set_cfgdata8(pi, PCIR_SUBCLASS, PCIS_SIMPLECOMM_OTHER);

	pci_emul_alloc_bar(pi, 0, PCIBAR_MEM64,
	    sizeof (struct pci_a2_bar));

	pci_emul_add_msixcap(pi, 2, 2);
	pci_emul_add_pciecap(pi, PCIEM_TYPE_ROOT_INT_EP);

	return (0);
}

static int
pci_a2_legacy_config(nvlist_t *nvl, const char *opts)
{
	char *tofree, *config, *name, *value;
	if (opts == NULL)
		return (0);
	config = tofree = strdup(opts);
	while ((name = strsep(&config, ",")) != NULL) {
		value = strchr(name, '=');
		if (value != NULL) {
			*value++ = '\0';
			set_config_value_node(nvl, name, value);
		}
	}
	free(tofree);
	return (0);
}

static void
pci_a2_write(struct vmctx *ctx, int vcpu, struct pci_devinst *pi,
    int baridx, uint64_t offset, int size, uint64_t value)
{
	struct pci_a2_softc *sc = pi->pi_arg;

	if (baridx == pci_msix_table_bar(pi) ||
	    baridx == pci_msix_pba_bar(pi)) {
		pci_emul_msix_twrite(pi, offset, size, value);
		return;
	}

	if (offset >= sizeof (struct pci_a2_bar))
		return;

	pthread_mutex_lock(&sc->sc_mtx);
	switch (offset) {
	case (offsetof(struct pci_a2_bar, b_flags)):
		if (value & EFLAG_RST) {
			if (sc->sc_workers_started) {
				port_send(sc->sc_ioport, PORTEV_A2_STOP, NULL);
				port_send(sc->sc_txport, PORTEV_A2_STOP, NULL);
				sc->sc_workers_started = 0;
				pthread_mutex_unlock(&sc->sc_mtx);
				pthread_join(sc->sc_tx_worker, NULL);
				pthread_join(sc->sc_io_worker, NULL);
				pthread_mutex_lock(&sc->sc_mtx);
				assert(!sc->sc_workers_started);
			}

			sc->sc_cbase = 0;
			sc->sc_cshift = 0;
			sc->sc_rbase = 0;
			sc->sc_rshift = 0;
			sc->sc_last_rdbell = 0xffffffff;
			sc->sc_last_cdbell = 0xffffffff;
			sc->sc_last_cpdbell = 0xffffffff;
			sc->sc_cpbase = 0;
			sc->sc_cpshift = 0;
			sc->sc_cmdr = NULL;
			sc->sc_maxcmd = 0;
			sc->sc_replyr = NULL;
			sc->sc_maxreply = 0;
			sc->sc_compr = NULL;
			sc->sc_maxcomp = 0;
			sc->sc_intr = 0;
			sc->sc_cpc = 0;

			sc->sc_ioport = port_create();
			sc->sc_txport = port_create();

			pthread_create(&sc->sc_io_worker, NULL,
			    pci_a2_io_worker, sc);
			pthread_setname_np(sc->sc_io_worker, "a2-io");
			pthread_create(&sc->sc_tx_worker, NULL,
			    pci_a2_tx_worker, sc);
			pthread_setname_np(sc->sc_io_worker, "a2-tx");
			sc->sc_workers_started = 1;

			sc->sc_eflags = 0;
		}
		break;
	case (offsetof(struct pci_a2_bar, b_cbase_lo)):
		if (sc->sc_eflags != 0)
			break;
		if (size == 8) {
			sc->sc_cbase = value;
		} else if (size == 4) {
			sc->sc_cbase =
			    (sc->sc_cbase & 0xffffffff00000000) | value;
		} else {
			break;
		}
		sc->sc_cmdr = paddr_guest2host(pi->pi_vmctx, sc->sc_cbase,
		    sizeof (struct pci_a2_descr) << sc->sc_cshift);
		break;
	case (offsetof(struct pci_a2_bar, b_cbase_hi)):
		if (sc->sc_eflags != 0)
			break;
		if (size != 4)
			break;
		sc->sc_cbase = (sc->sc_cbase & 0xffffffff) | (value << 32);
		sc->sc_cmdr = paddr_guest2host(pi->pi_vmctx, sc->sc_cbase,
		    sizeof (struct pci_a2_descr) << sc->sc_cshift);
		break;
	case (offsetof(struct pci_a2_bar, b_cshift)):
		if (sc->sc_eflags != 0)
			break;
		if (size != 4)
			break;
		sc->sc_cshift = value;
		sc->sc_maxcmd = 1 << value;
		sc->sc_cmdr = paddr_guest2host(pi->pi_vmctx, sc->sc_cbase,
		    sizeof (struct pci_a2_descr) << sc->sc_cshift);
		break;
	case (offsetof(struct pci_a2_bar, b_rbase_lo)):
		if (sc->sc_eflags != 0)
			break;
		if (size == 8) {
			sc->sc_rbase = value;
		} else if (size == 4) {
			sc->sc_rbase =
			    (sc->sc_rbase & 0xffffffff00000000) | value;
		} else {
			break;
		}
		sc->sc_replyr = paddr_guest2host(pi->pi_vmctx, sc->sc_rbase,
		    sizeof (struct pci_a2_descr) << sc->sc_rshift);
		break;
	case (offsetof(struct pci_a2_bar, b_rbase_hi)):
		if (sc->sc_eflags != 0)
			break;
		if (size != 4)
			break;
		sc->sc_rbase = (sc->sc_rbase & 0xffffffff) | (value << 32);
		sc->sc_replyr = paddr_guest2host(pi->pi_vmctx, sc->sc_rbase,
		    sizeof (struct pci_a2_descr) << sc->sc_rshift);
		break;
	case (offsetof(struct pci_a2_bar, b_rshift)):
		if (sc->sc_eflags != 0)
			break;
		if (size != 4)
			break;
		sc->sc_rshift = value;
		sc->sc_maxreply = 1 << value;
		sc->sc_replyr = paddr_guest2host(pi->pi_vmctx, sc->sc_rbase,
		    sizeof (struct pci_a2_descr) << sc->sc_rshift);
		break;
	case (offsetof(struct pci_a2_bar, b_cpbase_lo)):
		if (sc->sc_eflags != 0)
			break;
		if (size == 8) {
			sc->sc_cpbase = value;
		} else if (size == 4) {
			sc->sc_cpbase =
			    (sc->sc_cpbase & 0xffffffff00000000) | value;
		} else {
			break;
		}
		sc->sc_compr = paddr_guest2host(pi->pi_vmctx, sc->sc_cpbase,
		    sizeof (struct pci_a2_cdescr) << sc->sc_cpshift);
		A2DEBUG(sc, "wrote cpbase_lo, cpbase = %llx, compr = %p",
		    sc->sc_cpbase, sc->sc_compr);
		break;
	case (offsetof(struct pci_a2_bar, b_cpbase_hi)):
		if (sc->sc_eflags != 0)
			break;
		if (size != 4)
			break;
		sc->sc_cpbase = (sc->sc_cpbase & 0xffffffff) | (value << 32);
		sc->sc_compr = paddr_guest2host(pi->pi_vmctx, sc->sc_cpbase,
		    sizeof (struct pci_a2_cdescr) << sc->sc_cpshift);
		A2DEBUG(sc, "wrote cpbase_hi, cpbase = %llx, compr = %p",
		    sc->sc_cpbase, sc->sc_compr);
		break;
	case (offsetof(struct pci_a2_bar, b_cpshift)):
		if (sc->sc_eflags != 0)
			break;
		if (size != 4)
			break;
		sc->sc_cpshift = value;
		sc->sc_maxcomp = 1 << value;
		sc->sc_compr = paddr_guest2host(pi->pi_vmctx, sc->sc_cpbase,
		    sizeof (struct pci_a2_cdescr) << sc->sc_cpshift);
		A2DEBUG(sc, "wrote cpshift, cpbase = %llx, compr = %p",
		    sc->sc_cpbase, sc->sc_compr);
		break;
	case (offsetof(struct pci_a2_bar, b_dbell)):
		if (sc->sc_eflags != 0)
			break;
		if (size != 4)
			break;
		if (!sc->sc_workers_started) {
			A2DEBUG(sc, "workers not started at dbell write");
			pci_a2_error_locked(sc, EFLAG_SEQ);
			return;
		}
		if ((value & 0x80000000) && sc->sc_last_rdbell != value) {
			if (sc->sc_cbase == 0 || sc->sc_rbase == 0) {
				A2DEBUG(sc, "cbase/rbase not set at dbell "
				    "write");
				pci_a2_error_locked(sc, EFLAG_SEQ);
				return;
			}
			A2DEBUG(sc, "wakeup, reply dbell = %x", value);
			port_send(sc->sc_ioport, PORTEV_A2_WAKEUP, NULL);
			sc->sc_last_rdbell = value;
		} else if (sc->sc_last_cdbell != value) {
			if (sc->sc_cbase == 0 || sc->sc_rbase == 0) {
				A2DEBUG(sc, "cbase/rbase not set at dbell "
				    "write");
				pci_a2_error_locked(sc, EFLAG_SEQ);
				return;
			}
			A2DEBUG(sc, "wakeup, cmd dbell = %x", value);
			port_send(sc->sc_txport, PORTEV_A2_WAKEUP, NULL);
			sc->sc_last_cdbell = value;
		} else {
			A2DEBUG(sc, "ignored dbell, matches prev");
		}
		break;
	case (offsetof(struct pci_a2_bar, b_cpdbell)):
		if (sc->sc_eflags != 0)
			break;
		if (size != 4)
			break;
		if (!sc->sc_workers_started) {
			A2DEBUG(sc, "worker not started at cpdbell write");
			pci_a2_error_locked(sc, EFLAG_SEQ);
			return;
		}
		A2DEBUG(sc, "cpdbell write = %x", value);
		if (sc->sc_last_cpdbell != value)
			sc->sc_last_cpdbell = value;
		if (++value >= sc->sc_maxcomp)
			value = 0;
		if (value < sc->sc_cpc) {
			A2DEBUG(sc, "cpdbell write does not account for cpc");
			pci_generate_msix(sc->sc_pi, 0);
		}
		break;
	}
	pthread_mutex_unlock(&sc->sc_mtx);
}

static uint64_t
pci_a2_read(struct vmctx *ctx, int vcpu, struct pci_devinst *pi, int baridx,
    uint64_t offset, int size)
{
	struct pci_a2_softc *sc = pi->pi_arg;
	uint64_t value = 0;

	if (baridx == pci_msix_table_bar(pi) ||
	    baridx == pci_msix_pba_bar(pi)) {
		return (pci_emul_msix_tread(pi, offset, size));
	}

	pthread_mutex_lock(&sc->sc_mtx);
	switch (offset) {
	case (offsetof(struct pci_a2_bar, b_vmaj)):
		value = PCI_A2_VER_MAJ;
		break;
	case (offsetof(struct pci_a2_bar, b_vmin)):
		value = PCI_A2_VER_MIN;
		break;
	case (offsetof(struct pci_a2_bar, b_flags)):
		value = sc->sc_eflags;
		break;
	case (offsetof(struct pci_a2_bar, b_cbase_lo)):
		value = sc->sc_cbase;
		break;
	case (offsetof(struct pci_a2_bar, b_cbase_hi)):
		value = sc->sc_cbase >> 32;
		break;
	case (offsetof(struct pci_a2_bar, b_cshift)):
		value = sc->sc_cshift;
		break;
	case (offsetof(struct pci_a2_bar, b_rbase_lo)):
		value = sc->sc_rbase;
		break;
	case (offsetof(struct pci_a2_bar, b_rbase_hi)):
		value = sc->sc_rbase >> 32;
		break;
	case (offsetof(struct pci_a2_bar, b_rshift)):
		value = sc->sc_rshift;
		break;
	case (offsetof(struct pci_a2_bar, b_cpbase_lo)):
		value = sc->sc_cpbase;
		break;
	case (offsetof(struct pci_a2_bar, b_cpbase_hi)):
		value = sc->sc_cpbase >> 32;
		break;
	case (offsetof(struct pci_a2_bar, b_cpshift)):
		value = sc->sc_cpshift;
		break;
	case (offsetof(struct pci_a2_bar, b_dbell)):
		value = 0xdeadbeef;
		break;
	case (offsetof(struct pci_a2_bar, b_cpdbell)):
		value = 0xfeedface;
		break;
	default:
		value = 0xdefec8;
	}
	pthread_mutex_unlock(&sc->sc_mtx);

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

struct pci_devemu pci_de_a2 = {
	.pe_emu =	"a2",
	.pe_init =	pci_a2_init,
	.pe_legacy_config = pci_a2_legacy_config,
	.pe_barwrite =	pci_a2_write,
	.pe_barread =	pci_a2_read
};
PCI_EMUL_SET(pci_de_a2);
