#pragma once

#include <stdlib.h>

struct device;
typedef int gfp_t;
#define GFP_KERNEL 0

struct llist {
	void *data;
	struct llist *next;
	struct llist *prev;
};

struct llist llhead;

static inline void llist_add(struct llist *head, void *data)
{
	struct llist *cur = head;
	struct llist *next;
	// if the head is empty, add it here and return
	if(!cur->data)
	{
		cur->data = data;
		return;
	}

	while(cur->next)
		cur = cur->next;

	next = malloc(sizeof(*next));
	memset(next, 0, sizeof(*next));

	cur->next = next;
	next->prev = cur;
	next->data = data;
}

static inline void *devm_kzalloc(struct device *dev, size_t size, gfp_t gfp)
{
	void *p = malloc(size);
	memset(p, 0, size);

	llist_add(&llhead, p);

	return p;
}

static inline void devm_freeall()
{
	struct llist *cur = &llhead;
	struct llist *prev;

	while(cur->next)
		cur = cur->next;

	while(cur) {
		prev = cur->prev;
		free(cur->data);
		// only free ourselves if we aren't the head
		if(prev)
			free(cur);
		cur = prev;
	}
	// finally clear the head pointer
	memset(&llhead, 0, sizeof(llhead));
}
