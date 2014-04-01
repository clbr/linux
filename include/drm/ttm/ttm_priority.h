/**************************************************************************
 *
 * Copyright (c) 2014 Lauri Kasanen
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS, AUTHORS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 **************************************************************************/

#ifndef TTM_PRIORITY_H
#define TTM_PRIORITY_H

#include <linux/list.h>
#include <linux/rbtree.h>
#include <linux/types.h>

/**
 * struct ttm_pqueue - zero-initialize it when allocating
 */

struct ttm_pqueue {
	struct rb_root tree;
};

struct ttm_pqueue_entry {
	u64 score;
	struct rb_node node;
	struct list_head list;
};

void ttm_prio_init_entry(struct ttm_pqueue_entry * const entry);

/**
 * ttm_prio_add - add this entry to the priority queue.
 * Set the score before calling this.
 */

void ttm_prio_add(struct ttm_pqueue * const queue,
		  struct ttm_pqueue_entry * const entry);

/**
 * ttm_prio_query_lowest - return the entry with the lowest score.
 *
 * Returns NULL if there are none.
 */

struct ttm_pqueue_entry *ttm_prio_query_lowest(const struct ttm_pqueue * const queue);

/**
 * ttm_prio_remove - remove a previously queried entry from the queue.
 */

void ttm_prio_remove(struct ttm_pqueue * const queue,
		     struct ttm_pqueue_entry * const entry);

/**
 * ttm_prio_query_next - walk the queue
 *
 * Returns NULL if there is no next entry.
 */

struct ttm_pqueue_entry *ttm_prio_query_next(struct ttm_pqueue_entry * const entry);

/**
 * ttm_prio_is_queued - test if an entry is in the queue.
 *
 * Returns 1 if it is, 0 if not.
 */

int ttm_prio_is_queued(const struct ttm_pqueue_entry * const entry);

#endif
