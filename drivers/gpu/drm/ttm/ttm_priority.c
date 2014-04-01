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


#include <drm/ttm/ttm_priority.h>
#include <linux/bug.h>

static void ttm_prio_insert_rb(struct rb_root * const root,
			       struct ttm_pqueue_entry * const data,
			       struct rb_node *parent,
			       struct rb_node **where)
{
	BUG_ON(!where || (!parent && root->rb_node));

	rb_link_node(&data->node, parent, where);
	rb_insert_color(&data->node, root);
}

static struct ttm_pqueue_entry *ttm_prio_search_rb(struct rb_root *root,
						   u64 score,
						   struct rb_node **parent,
						   struct rb_node ***where)
{
	struct rb_node **new = &root->rb_node;

	while (*new) {
		struct ttm_pqueue_entry *this = container_of(*new,
							     struct ttm_pqueue_entry,
							     node);

		*parent = *new;

		if (score < this->score)
			new = &((*new)->rb_left);
		else if (score > this->score)
			new = &((*new)->rb_right);
		else
			return this;

		BUG_ON(*parent == *new);
	}

	*where = new;

	return NULL;
}

void ttm_prio_add(struct ttm_pqueue * const queue,
		  struct ttm_pqueue_entry * const entry)
{
	struct rb_root * const tree = &queue->tree;
	struct rb_node **place = NULL, *parent = NULL;
	struct ttm_pqueue_entry *test;

	if (ttm_prio_is_queued(entry))
		ttm_prio_remove(queue, entry);

	test = ttm_prio_search_rb(tree, entry->score, &parent, &place);

	if (!test)
		ttm_prio_insert_rb(tree, entry, parent, place);
	else
		list_add_tail(&entry->list, &test->list);
}

struct ttm_pqueue_entry *ttm_prio_query_lowest(const struct ttm_pqueue * const queue)
{
	const struct rb_root * const root = &queue->tree;
	struct rb_node *node;

	node = rb_first(root);
	if (!node)
		return NULL;

	return container_of(node, struct ttm_pqueue_entry, node);
}

void ttm_prio_remove(struct ttm_pqueue * const queue,
		     struct ttm_pqueue_entry * const entry)
{
	struct rb_root * const tree = &queue->tree;

	if (list_empty(&entry->list)) {
		rb_erase(&entry->node, tree);
		RB_CLEAR_NODE(&entry->node);
	} else {
		struct ttm_pqueue_entry *next = list_first_entry(&entry->list,
								 struct ttm_pqueue_entry,
								 list);
		if (RB_EMPTY_NODE(&next->node) && !RB_EMPTY_NODE(&entry->node))
			rb_replace_node(&entry->node, &next->node, tree);

		list_del_init(&entry->list);
		RB_CLEAR_NODE(&entry->node);
	}
}

void ttm_prio_init_entry(struct ttm_pqueue_entry * const entry)
{
	entry->score = 0;
	RB_CLEAR_NODE(&entry->node);
	INIT_LIST_HEAD(&entry->list);
}

struct ttm_pqueue_entry *ttm_prio_query_next(struct ttm_pqueue_entry * const entry)
{
	struct ttm_pqueue_entry *next = list_next_entry(entry, list);
	struct rb_node *node;

	if (list_empty(&entry->list)) {
		node = rb_next(&entry->node);
		if (!node)
			return NULL;
		return container_of(node, struct ttm_pqueue_entry, node);
	} else if (!RB_EMPTY_NODE(&next->node)) {
		node = rb_next(&next->node);
		if (!node)
			return NULL;
		return container_of(node, struct ttm_pqueue_entry, node);
	} else {
		return next;
	}
}

int ttm_prio_is_queued(const struct ttm_pqueue_entry * const entry)
{
	return !(list_empty(&entry->list) && RB_EMPTY_NODE(&entry->node));
}
