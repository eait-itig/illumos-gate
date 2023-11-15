/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License (the "License").
 * You may not use this file except in compliance with the License.
 *
 * You can obtain a copy of the license at usr/src/OPENSOLARIS.LICENSE
 * or http://www.opensolaris.org/os/licensing.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file at usr/src/OPENSOLARIS.LICENSE.
 * If applicable, add the following below this CDDL HEADER, with the
 * fields enclosed by brackets "[]" replaced with your own identifying
 * information: Portions Copyright [yyyy] [name of copyright owner]
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2008 Sun Microsystems, Inc.  All rights reserved.
 * Use is subject to license terms.
 */

#include <sys/types.h>
#include <sys/rwlock.h>
#include <mdb/mdb_modapi.h>
#include <mdb/mdb_ctf.h>
#include <sys/neti.h>


/*
 * PROT_LENGTH is the max length. If the true length is bigger
 * it is truncated.
 */
#define	PROT_LENGTH 32

typedef struct {
	const char	*ctf_name;
	mdb_ctf_id_t	 ctf_id;
	size_t		 ctf_base;
} ctf_struct_t;

struct o_netstack {
	ctf_struct_t	ctf;
	size_t		o_netstack_u;
	struct {
		ctf_struct_t	ctf;
		size_t		o_nu_s;
		struct {
			ctf_struct_t 	ctf;
			size_t		o_nu_neti;
		} nu_s;
	} netstack_u;
};
static struct o_netstack	o_netstack = {
	.ctf = 		{ .ctf_name	= "struct netstack" }
};

struct o_neti_stack {
	ctf_struct_t	ctf;
	size_t		o_nts_netd_head;
	struct {
		ctf_struct_t	ctf;
		size_t		o_lh_first;
	} nts_netd_head;
};
static struct o_neti_stack	o_neti_stack = {
	.ctf = 		{ .ctf_name	= "struct neti_stack_s" }
};

struct o_net_data {
	ctf_struct_t	ctf;
	struct {
		ctf_struct_t	ctf;
		size_t		o_netp_name;
	} netd_info;
	size_t		o_netd_info;
	size_t		o_netd_hooks;
	size_t		o_netd_list;
	struct {
		ctf_struct_t	ctf;
		size_t		o_le_next;
	} netd_list;
};
static struct o_net_data	o_net_data = {
	.ctf = 		{ .ctf_name	= "struct net_data" }
};

static int
lookup_struct(ctf_struct_t *ctf)
{
	if (mdb_ctf_lookup_by_name(ctf->ctf_name, &ctf->ctf_id) == -1) {
		mdb_warn("couldn't find type %s", ctf->ctf_name);
		return (-1);
	}
	return (0);
}
#define	LOOKUP_STRUCT(st)		\
	lookup_struct(&st.ctf)

static int
lookup_member(ctf_struct_t *ctf, const char *name, ulong_t *poff)
{
	if (mdb_ctf_offsetof(ctf->ctf_id, name, poff)) {
		mdb_warn("couldn't find member %s of %s", name, ctf->ctf_name);
		return (-1);
	}
	if ((*poff) % 8 != 0) {
		mdb_warn("member %s of type %s is unsupported bitfield",
		    name, ctf->ctf_name);
		return (DCMD_ERR);
	}
	*poff /= 8;
	*poff += ctf->ctf_base;
	return (0);
}
#define	LOOKUP_MEMBER(st, memb) 	\
	lookup_member(&st.ctf, #memb, &st.o_ ## memb)

static int
lookup_struct_member(ctf_struct_t *ctf, const char *name, ctf_struct_t *mctf,
    size_t *poff)
{
	mctf->ctf_name = name;
	if (mdb_ctf_member_info(ctf->ctf_id, mctf->ctf_name, poff,
	    &mctf->ctf_id)) {
		mdb_warn("couldn't find member %s of %s", mctf->ctf_name,
		    ctf->ctf_name);
		return (-1);
	}
	if ((*poff) % 8 != 0) {
		mdb_warn("member %s of type %s is unsupported bitfield",
		    name, ctf->ctf_name);
		return (DCMD_ERR);
	}
	*poff /= 8;
	*poff += ctf->ctf_base;
	mctf->ctf_base = *poff;
	return (0);
}
#define	LOOKUP_SMEMBER(st, memb)	\
	lookup_struct_member(&st.ctf, #memb, &st.memb.ctf, &st.o_ ## memb)

static int
load_offsets(void)
{
	if (LOOKUP_STRUCT(o_netstack) ||
	    LOOKUP_SMEMBER(o_netstack, netstack_u) ||
	    LOOKUP_SMEMBER(o_netstack.netstack_u, nu_s) ||
	    LOOKUP_MEMBER(o_netstack.netstack_u.nu_s, nu_neti)) {
		return (-1);
	}
	if (LOOKUP_STRUCT(o_neti_stack) ||
	    LOOKUP_SMEMBER(o_neti_stack, nts_netd_head) ||
	    LOOKUP_MEMBER(o_neti_stack.nts_netd_head, lh_first)) {
		return (-1);
	}
	if (LOOKUP_STRUCT(o_net_data) ||
	    LOOKUP_MEMBER(o_net_data, netd_info) ||
	    LOOKUP_MEMBER(o_net_data, netd_hooks) ||
	    LOOKUP_SMEMBER(o_net_data, netd_info) ||
	    LOOKUP_MEMBER(o_net_data.netd_info, netp_name) ||
	    LOOKUP_SMEMBER(o_net_data, netd_list) ||
	    LOOKUP_MEMBER(o_net_data.netd_list, le_next)) {
		return (-1);
	}

	return (0);
}

/*
 * List pfhooks netinfo information.
 */
/*ARGSUSED*/
int
netinfolist(uintptr_t netstack, uint_t flags, int argc, const mdb_arg_t *argv)
{
	uintptr_t neti_stack, net_data, strp, netd_hooks;
	uintptr_t next;
	char str[PROT_LENGTH];

	if (argc)
		return (DCMD_USAGE);

	if (load_offsets())
		return (DCMD_ERR);

	if (mdb_vread(&neti_stack, sizeof (neti_stack),
	    netstack + o_netstack.netstack_u.nu_s.o_nu_neti) == -1) {
		mdb_warn("couldn't read netstack_neti from netstack at %p",
		    (void *)netstack);
		return (DCMD_ERR);
	}

	if (mdb_vread(&net_data, sizeof (net_data),
	    neti_stack + o_neti_stack.nts_netd_head.o_lh_first) == -1) {
		mdb_warn("couldn't read netd list head from neti_stack at %p",
		    (void *)neti_stack);
		return (DCMD_ERR);
	}
	mdb_printf("%<u>%?s %?s %10s%</u>\n",
	    "ADDR(net_proto)", "ADDR(hookevent)", "netinfo");

	while (net_data != 0) {
		if (mdb_vread(&strp, sizeof (strp),
		    net_data + o_net_data.netd_info.o_netp_name) == -1) {
			mdb_warn("couldn't read netp_name of %p",
			    (void *)net_data);
			return (DCMD_ERR);
		}
		if (strp == 0) {
			mdb_warn("netinfo at %p has null protocol",
			    (void *)(net_data + o_net_data.o_netd_info));
			return (DCMD_ERR);
		}
		if (mdb_readstr(str, sizeof (str), strp) == -1) {
			mdb_warn("couldn't read protocol at %p", (void *)strp);
			return (DCMD_ERR);
		}

		if (mdb_vread(&netd_hooks, sizeof (netd_hooks),
		    net_data + o_net_data.o_netd_hooks) == -1) {
			mdb_warn("couldn't read netd_hooks of %p",
			    (void *)net_data);
			return (DCMD_ERR);
		}

		mdb_printf("%0?p %0?p %10s\n",
		    net_data + o_net_data.o_netd_info,
		    netd_hooks,
		    str);

		if (mdb_vread(&next, sizeof (next),
		    net_data + o_net_data.netd_list.o_le_next) == -1) {
			mdb_warn("couldn't read le_next of %p",
			    (void *)net_data);
			return (DCMD_ERR);
		}
		net_data = next;
	}

	return (DCMD_OK);
}

static const mdb_dcmd_t dcmds[] = {
	{ "netinfolist", "", "display netinfo information",
		netinfolist, NULL },
	{ NULL }
};

static const mdb_modinfo_t modinfo = { MDB_API_VERSION, dcmds };

const mdb_modinfo_t *
_mdb_init(void)
{
	return (&modinfo);
}
