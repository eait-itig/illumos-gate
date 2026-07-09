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
#include <inet/ip.h>
#include <sys/hook.h>
#include <sys/hook_impl.h>

#define	MAX_LENGTH 64

typedef struct {
	const char	*ctf_name;
	mdb_ctf_id_t	 ctf_id;
	size_t		 ctf_base;
} ctf_struct_t;

struct o_list_node {
	ctf_struct_t	ctf;
	size_t		o_list_next;
	size_t		o_list_prev;
};

struct o_list_head {
	ctf_struct_t	ctf;
	size_t 		o_list_offset;
	size_t		o_list_head;
	struct o_list_node	list_head;
};

struct o_netstack {
	ctf_struct_t	ctf;
	size_t		o_netstack_u;
	struct {
		ctf_struct_t	ctf;
		size_t		o_nu_s;
		struct {
			ctf_struct_t 	ctf;
			size_t		o_nu_hook;
		} nu_s;
	} netstack_u;
};
static struct o_netstack	o_netstack = {
	.ctf = 		{ .ctf_name	= "struct netstack" }
};

struct o_hook_stack {
	ctf_struct_t	ctf;
	size_t		o_hks_familylist;
	struct o_list_head	hks_familylist;
};
static struct o_hook_stack	o_hook_stack = {
	.ctf =		{ .ctf_name	= "struct hook_stack" }
};

struct o_hook_family_int {
	ctf_struct_t	ctf;
	size_t		o_hfi_node;
	struct o_list_node	hfi_node;
	size_t		o_hfi_head;
	struct o_list_head	hfi_head;
	size_t		o_hfi_family;
	struct {
		ctf_struct_t	ctf;
		size_t		o_hf_name;
	} hfi_family;
};
static struct o_hook_family_int	o_hook_family_int = {
	.ctf =		{ .ctf_name = "struct hook_family_int" }
};

struct o_hook_event_int {
	ctf_struct_t	ctf;
	size_t		o_hei_node;
	struct o_list_node	hei_node;
	size_t		o_hei_event;
	size_t		o_hei_head;
	struct o_list_head	hei_head;
};
static struct o_hook_event_int	o_hook_event_int = {
	.ctf =		{ .ctf_name = "struct hook_event_int" }
};

struct o_hook_event {
	ctf_struct_t	ctf;
	size_t		o_he_name;
	size_t		o_he_flags;
	size_t		o_he_interested;
};
static struct o_hook_event	o_hook_event = {
	.ctf =		{ .ctf_name = "struct hook_event_s" }
};

struct o_hook_int {
	ctf_struct_t	ctf;
	size_t		o_hi_node;
	struct o_list_node	hi_node;
	size_t		o_hi_hook;
	struct {
		ctf_struct_t	ctf;
		size_t		o_h_func;
		size_t		o_h_name;
		size_t		o_h_flags;
		size_t		o_h_hint;
		size_t		o_h_hintvalue;
	} hi_hook;
};
static struct o_hook_int	o_hook_int = {
	.ctf =		{ .ctf_name = "struct hook_int" }
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
	    LOOKUP_MEMBER(o_netstack.netstack_u.nu_s, nu_hook)) {
		return (-1);
	}

	if (LOOKUP_STRUCT(o_hook_family_int) ||
	    LOOKUP_SMEMBER(o_hook_family_int, hfi_node) ||
	    LOOKUP_MEMBER(o_hook_family_int.hfi_node, list_next) ||
	    LOOKUP_SMEMBER(o_hook_family_int, hfi_head) ||
	    LOOKUP_MEMBER(o_hook_family_int.hfi_head, list_offset) ||
	    LOOKUP_SMEMBER(o_hook_family_int.hfi_head, list_head) ||
	    LOOKUP_MEMBER(o_hook_family_int.hfi_head.list_head, list_next) ||
	    LOOKUP_SMEMBER(o_hook_family_int, hfi_family) ||
	    LOOKUP_MEMBER(o_hook_family_int.hfi_family, hf_name)) {
		return (-1);
	}
	if (LOOKUP_STRUCT(o_hook_event_int) ||
	    LOOKUP_SMEMBER(o_hook_event_int, hei_node) ||
	    LOOKUP_MEMBER(o_hook_event_int.hei_node, list_next) ||
	    LOOKUP_SMEMBER(o_hook_event_int, hei_head) ||
	    LOOKUP_MEMBER(o_hook_event_int.hei_head, list_offset) ||
	    LOOKUP_SMEMBER(o_hook_event_int.hei_head, list_head) ||
	    LOOKUP_MEMBER(o_hook_event_int.hei_head.list_head, list_next)) {
		return (-1);
	}
	if (LOOKUP_STRUCT(o_hook_stack) ||
	    LOOKUP_SMEMBER(o_hook_stack, hks_familylist) ||
	    LOOKUP_MEMBER(o_hook_stack.hks_familylist, list_offset) ||
	    LOOKUP_SMEMBER(o_hook_stack.hks_familylist, list_head) ||
	    LOOKUP_MEMBER(o_hook_stack.hks_familylist.list_head, list_next)) {
		return (-1);
	}
	if (LOOKUP_STRUCT(o_hook_int) ||
	    LOOKUP_SMEMBER(o_hook_int, hi_node) ||
	    LOOKUP_MEMBER(o_hook_int.hi_node, list_next) ||
	    LOOKUP_SMEMBER(o_hook_int, hi_hook) ||
	    LOOKUP_MEMBER(o_hook_int.hi_hook, h_func) ||
	    LOOKUP_MEMBER(o_hook_int.hi_hook, h_name) ||
	    LOOKUP_MEMBER(o_hook_int.hi_hook, h_flags) ||
	    LOOKUP_MEMBER(o_hook_int.hi_hook, h_hint) ||
	    LOOKUP_MEMBER(o_hook_int.hi_hook, h_hintvalue)) {
		return (-1);
	}

	if (LOOKUP_STRUCT(o_hook_event) ||
	    LOOKUP_MEMBER(o_hook_event, he_name) ||
	    LOOKUP_MEMBER(o_hook_event, he_flags) ||
	    LOOKUP_MEMBER(o_hook_event, he_interested)) {
		return (-1);
	}

	return (0);
}

/*
 * List pfhooks hook list information.
 */
/*ARGSUSED*/
int
hooklist(uintptr_t hei, uint_t flags, int argc, const mdb_arg_t *argv)
{
	uintptr_t hi, strp, next, lhead, hintv, func;
	size_t loff;
	char hrstr[MAX_LENGTH];
	char hvstr[MAX_LENGTH];
	GElf_Sym sym;
	char buf[MDB_SYM_NAMLEN + 1];
	hook_hint_t hint;
	uint_t hflags;
	const char *hintname;

	if (argc)
		return (DCMD_USAGE);

	if (load_offsets())
		return (DCMD_ERR);

	if (mdb_vread(&loff, sizeof (loff),
	    hei + o_hook_event_int.hei_head.o_list_offset) == -1) {
		mdb_warn("couldn't read hook register at %p", hei);
		return (DCMD_ERR);
	}
	if (loff != o_hook_int.o_hi_node) {
		mdb_warn("bad list offset (%lx, expected %lx) in hook "
		    "register at %p", loff, o_hook_int.o_hi_node, hei);
		return (DCMD_ERR);
	}
	lhead = hei + o_hook_event_int.hei_head.o_list_head;
	if (mdb_vread(&next, sizeof (next),
	    hei + o_hook_event_int.hei_head.list_head.o_list_next) == -1) {
		mdb_warn("couldn't read hook register at %p", hei);
		return (DCMD_ERR);
	}

	mdb_printf("%<u>%?s %8s %20s %4s %24s %24s%</u>\n",
	    "ADDR", "FLAG", "FUNC", "HINT", "NAME", "HINTVALUE");
	while (next != 0 && next != lhead) {
		hi = next - loff;
		if (mdb_vread(&hflags, sizeof (hflags),
		    hi + o_hook_int.hi_hook.o_h_flags) == -1) {
			mdb_warn("couldn't read hook list at %p", (void *)hi);
			return (DCMD_ERR);
		}
		if (mdb_vread(&hint, sizeof (hint),
		    hi + o_hook_int.hi_hook.o_h_hint) == -1) {
			mdb_warn("couldn't read hook list at %p", (void *)hi);
			return (DCMD_ERR);
		}
		if (mdb_vread(&hintv, sizeof (hintv),
		    hi + o_hook_int.hi_hook.o_h_hintvalue) == -1) {
			mdb_warn("couldn't read hook list at %p", (void *)hi);
			return (DCMD_ERR);
		}
		if (mdb_vread(&func, sizeof (func),
		    hi + o_hook_int.hi_hook.o_h_func) == -1) {
			mdb_warn("couldn't read hook list at %p", (void *)hi);
			return (DCMD_ERR);
		}
		if (mdb_vread(&strp, sizeof (strp),
		    hi + o_hook_int.hi_hook.o_h_name) == -1) {
			mdb_warn("couldn't read hook list at %p", (void *)hi);
			return (DCMD_ERR);
		}
		if (strp == 0) {
			mdb_warn("hook list at %p has null role", (void *)hi);
			return (DCMD_ERR);
		}
		if (mdb_readstr(hrstr, sizeof (hrstr), strp) == -1) {
			mdb_warn("couldn't read list role at %p", strp);
			return (DCMD_ERR);
		}
		switch (hint) {
		case HH_BEFORE :
		case HH_AFTER :
			if (mdb_readstr(hvstr, sizeof (hvstr), hintv) == -1) {
				mdb_warn("couldn't read hintvalue at %p",
				    hintv);
				return (DCMD_ERR);
			}
			hintname = hvstr;
			break;
		default :
			hintname = "";
			break;
		}
		if (mdb_lookup_by_addr(func, MDB_SYM_EXACT,
		    buf, sizeof (buf), &sym) == -1) {
			mdb_printf("%0?p %8x %0?p %4d %24s %24s\n",
			    hi, hflags, func, hint, hrstr, hintname);
		} else {
			mdb_printf("%0?p %8x %20s %4d %24s %24s\n",
			    hi, hflags, buf, hint, hrstr, hintname);
		}
		if (mdb_vread(&next, sizeof (next),
		    hi + o_hook_int.hi_node.o_list_next) == -1) {
			mdb_warn("couldn't read hook list at %p", hi);
			return (DCMD_ERR);
		}
	}
	return (DCMD_OK);
}

/*
 * List pfhooks event information.
 * List the hooks information in verbose mode as well.
 */
/*ARGSUSED*/
int
hookeventlist(uintptr_t hfi, uint_t flags, int argc, const mdb_arg_t *argv)
{
	uintptr_t hei, next, strp, he, lhead;
	size_t loff;
	char hprstr[MAX_LENGTH];
	int heflags;

	if (argc)
		return (DCMD_USAGE);

	if (load_offsets())
		return (DCMD_ERR);

	if (mdb_vread(&loff, sizeof (loff),
	    hfi + o_hook_family_int.hfi_head.o_list_offset) == -1) {
		mdb_warn("couldn't read hook family at %p", hfi);
		return (DCMD_ERR);
	}
	if (loff != o_hook_event_int.o_hei_node) {
		mdb_warn("bad list offset (%lx, expected %lx) in hook family "
		    "at %p", loff, o_hook_event_int.o_hei_node, hfi);
		return (DCMD_ERR);
	}
	lhead = hfi + o_hook_family_int.hfi_head.o_list_head;
	if (mdb_vread(&next, sizeof (next),
	    hfi + o_hook_family_int.hfi_head.list_head.o_list_next) == -1) {
		mdb_warn("couldn't read hook family at %p", hfi);
		return (DCMD_ERR);
	}

	mdb_printf("%<u>%?s %10s %20s%</u>\n", "ADDR", "FLAG", "NAME");
	while (next != 0 && next != lhead) {
		hei = next - loff;
		if (mdb_vread(&he, sizeof (he),
		    hei + o_hook_event_int.o_hei_event)) {
			mdb_warn("couldn't read hook register at %p", hei);
			return (DCMD_ERR);
		}
		if (he == 0) {
			mdb_warn("hook register at %p has no hook provider",
			    hei);
			return (DCMD_ERR);
		}
		if (mdb_vread(&strp, sizeof (strp),
		    he + o_hook_event.o_he_name) == -1) {
			mdb_warn("couldn't read hook at %p", he);
			return (DCMD_ERR);
		}
		if (strp == 0) {
			mdb_warn("hook provider at %p has null role", he);
			return (DCMD_ERR);
		}
		if (mdb_readstr(hprstr, sizeof (hprstr), strp) == -1) {
			mdb_warn("couldn't read provider role at %p", strp);
			return (DCMD_ERR);
		}
		if (mdb_vread(&heflags, sizeof (heflags),
		    he + o_hook_event.o_he_flags) == -1) {
			mdb_warn("couldn't read hook at %p", he);
			return (DCMD_ERR);
		}
		mdb_printf("%0?p %10x %20s\n", hei, heflags, hprstr);
		if (mdb_vread(&next, sizeof (next),
		    hei + o_hook_event_int.hei_node.o_list_next) == -1) {
			mdb_warn("couldn't read hook register at %p", hei);
			return (DCMD_ERR);
		}
	}

	return (DCMD_OK);
}

/*
 * List pfhooks family information.
 */
/*ARGSUSED*/
int
hookrootlist(uintptr_t netstack, uint_t flags, int argc, const mdb_arg_t *argv)
{
	uintptr_t hook_stack, hfi, next, lhead, strp;
	size_t loff;
	char hrrstr[MAX_LENGTH];

	if (argc)
		return (DCMD_USAGE);

	if (load_offsets())
		return (DCMD_ERR);

	if (mdb_vread(&hook_stack, sizeof (hook_stack),
	    netstack + o_netstack.netstack_u.nu_s.o_nu_hook) == -1) {
		mdb_warn("couldn't read netstack_hook from netstack at %p",
		    (void *)netstack);
		return (DCMD_ERR);
	}

	if (hook_stack == 0) {
		mdb_warn("netstack_hook is null on netstack at %p",
		    (void *)netstack);
		return (DCMD_ERR);
	}

	if (mdb_vread(&loff, sizeof (loff),
	    hook_stack + o_hook_stack.hks_familylist.o_list_offset) == -1) {
		mdb_warn("couldn't read list_offset from hook_stack at %p",
		    (void *)hook_stack);
		return (DCMD_ERR);
	}
	if (loff != o_hook_family_int.o_hfi_node) {
		mdb_warn("bad list offset (%lx, expected %lx) in hook "
		    "stack at %p", loff, o_hook_family_int.o_hfi_node,
		    hook_stack);
		return (DCMD_ERR);
	}
	lhead = hook_stack + o_hook_stack.hks_familylist.o_list_head;
	if (mdb_vread(&next, sizeof (next),
	    hook_stack + o_hook_stack.hks_familylist.list_head.o_list_next)) {
		mdb_warn("couldn't read list_next from hook_stack at %p",
		    (void *)hook_stack);
		return (DCMD_ERR);
	}

	mdb_printf("%<u>%?s %10s%</u>\n", "ADDR", "FAMILY");
	while (next != 0 && next != lhead) {
		hfi = next - loff;
		if (mdb_vread(&strp, sizeof (strp),
		    hfi + o_hook_family_int.hfi_family.o_hf_name) == -1) {
			mdb_warn("couldn't read hf_fname from hook family at "
			    "%p", (void *)hfi);
			return (DCMD_ERR);
		}
		if (strp == 0) {
			mdb_warn("hook root at %p has null role", (void *)hfi);
			return (DCMD_ERR);
		}
		if (mdb_readstr(hrrstr, sizeof (hrrstr), strp) == -1) {
			mdb_warn("couldn't read root role at %p", (void *)strp);
			return (DCMD_ERR);
		}
		mdb_printf("%0?p %10s\n", hfi, hrrstr);
		if (mdb_vread(&next, sizeof (next),
		    hfi + o_hook_family_int.hfi_node.o_list_next) == -1) {
			mdb_warn("couldn't read list_next from hook family at "
			    "%p", (void *)hfi);
			return (DCMD_ERR);
		}
	}

	return (DCMD_OK);
}

typedef struct {
	size_t		hsw_loff;
	uintptr_t	hsw_lhead;
	uintptr_t	hsw_next;
} hookevent_stack_walk_t;

static int
hookevent_stack_walk_init(mdb_walk_state_t *wsp)
{
	hookevent_stack_walk_t *hsw;

	if (load_offsets())
		return (DCMD_ERR);

	if (wsp->walk_addr == 0) {
		mdb_warn("global walk not supported\n");
		return (WALK_ERR);
	}

	hsw = mdb_zalloc(sizeof (*hsw), UM_SLEEP | UM_GC);

	if (mdb_vread(&hsw->hsw_loff, sizeof (hsw->hsw_loff),
	    wsp->walk_addr + o_hook_family_int.hfi_head.o_list_offset) == -1) {
		mdb_warn("couldn't read list_offset from hook family at %p",
		    (void *)wsp->walk_addr);
		return (DCMD_ERR);
	}
	if (hsw->hsw_loff != o_hook_event_int.o_hei_node) {
		mdb_warn("bad list offset (%lx, expected %lx) in hook family "
		    "at %p", hsw->hsw_loff, o_hook_event_int.o_hei_node,
		    wsp->walk_addr);
		return (DCMD_ERR);
	}
	hsw->hsw_lhead = wsp->walk_addr +
	    o_hook_family_int.hfi_head.o_list_head;
	if (mdb_vread(&hsw->hsw_next, sizeof (hsw->hsw_next),
	    wsp->walk_addr +
	    o_hook_family_int.hfi_head.list_head.o_list_next) == -1) {
		mdb_warn("couldn't read list_next from hook family at %p",
		    wsp->walk_addr);
		return (DCMD_ERR);
	}
	wsp->walk_addr = hsw->hsw_next - hsw->hsw_loff;
	return (wsp->walk_callback(wsp->walk_addr, NULL,
	    wsp->walk_cbdata));
}

static int
hookevent_stack_walk_step(mdb_walk_state_t *wsp)
{
	hookevent_stack_walk_t *hsw;

	hsw = wsp->walk_data;

	if (mdb_vread(&hsw->hsw_next, sizeof (hsw->hsw_next),
	    wsp->walk_addr + o_hook_event_int.hei_node.o_list_next) == -1) {
		mdb_warn("couldn't read list_next from hook_event_int at %p",
		    wsp->walk_addr);
		return (DCMD_ERR);
	}
	if (hsw->hsw_next == 0 || hsw->hsw_next == hsw->hsw_lhead)
		return (WALK_DONE);

	wsp->walk_addr = hsw->hsw_next - hsw->hsw_loff;
	return (wsp->walk_callback(wsp->walk_addr, NULL,
	    wsp->walk_cbdata));
}

static const mdb_dcmd_t dcmds[] = {
	{ "hookrootlist", "", "display hook families on a netstack_t",
	    hookrootlist },
	{ "hookeventlist", "",
	    "display hook event info from a hook_family_int_t",
	    hookeventlist, NULL },
	{ "hooklist", "", "display hooks on a hook_event_int_t", hooklist },
	{ NULL }
};

static const mdb_walker_t walkers[] = {
	{ "hookevent_stack", "walk hook events on a hook_family_int_t",
		hookevent_stack_walk_init, hookevent_stack_walk_step, NULL },
	{ NULL }
};

static const mdb_modinfo_t modinfo = { MDB_API_VERSION, dcmds, walkers };

const mdb_modinfo_t *
_mdb_init(void)
{
	return (&modinfo);
}
