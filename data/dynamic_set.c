// dynamic set implementation
// by Mark Hill

#include<stdint.h>
#include<stdlib.h>
#include<stdio.h>
#include<pthread.h>

#include<dynamic_set.h>


/*
 * internal representation of a dynamic set
 * @data		array of void * pointers
 * @size		size of the memory measured in void * pointers
 * @count		number of stored elements in the array
 */
struct dyn_set {
	pthread_mutex_t lock;
	int64_t size;
	int64_t count;
	void **data;
};


/*
 * internal function to resize a set
 * @multiple		resizes set to (multiple) * set->size if multiple >= 0, else set->size / multiple
 *
 * @return			0 on success
 * 					1 on failure
 */
static inline int8_t __dyn_set_resize(struct dyn_set *set, int8_t multiple) {
	if (dyn_set_lock(set))
		return 1;

	void **temp;
	uint64_t newsize = (multiple >= 0) ? set->size * multiple : set->size / (-1 * multiple);
	if ((temp = realloc(set->data, sizeof(void *) * newsize))) {
		set->data = temp;
		printf("DEBUG: resized array %lx from size %lu to %lu\n", (uint64_t)set->data, set->size, newsize);
		set->size = newsize;
		return dyn_set_unlock(set);
	} else {
		printf("DEBUG: error resizing array %lx from size %lu to %lu\n", (uint64_t)set->data, set->size, newsize);
		dyn_set_unlock(set);
		return 1;
	}
}

#define __GROWTH_FACTOR 2
/*
 * internal function to grow a set
 * @return		0 on success
 * 				1 on failure
 */
static int8_t __dyn_set_grow(struct dyn_set *set) {
	if (set->count < set->size)
		return 0;
	return __dyn_set_resize(set, __GROWTH_FACTOR);
}

/*
 * internal function to shrink a set
 * @return		0 on success
 * 				1 on failure
 */
static int8_t __dyn_set_shrink(struct dyn_set *set) {
	if (set->count * __GROWTH_FACTOR * __GROWTH_FACTOR > set->size)
		return 0;
	return __dyn_set_resize(set, -1 * __GROWTH_FACTOR);
}

/*
 * internal function to find the index of an item in a set
 * @return		the index of the object in the set->data array
 * 				set->count if the object was not found
 * 				set->count + 1 if any of the inputs are invalid (NULL)
 */
static uint64_t __dyn_set_find(struct dyn_set *set, void *item) {
	if (dyn_set_lock(set))
		return 1;

	if (set == NULL || set->data == NULL || item == NULL) {
		printf("DEBUG: NULL item, set, or set->data in ___dyn_set_find()\n");
		dyn_set_unlock(set);
		return set->count + 1;
	}

	for (int64_t i = 0; i < set->count; i++) {
		if (set->data[i] == item) {
			printf("DEBUG: found item %lx at index %lu\n", (uint64_t)item, i);
			dyn_set_unlock(set);
			return i;
		}
	}

	printf("DEBUG: could not find item %lx in set %lx\n", (uint64_t)item, (uint64_t)set);
	dyn_set_unlock(set);
	return set->count;
}



struct dyn_set *dyn_set_init(uint64_t start_size) {
	if (start_size <= 1) {
		printf("DEBUG: requested 0 size dynamic set - returning NULL\n");
		return NULL;
	}

	struct dyn_set *set;
	pthread_mutexattr_t attr;
	if (!(set = malloc(sizeof(struct dyn_set)))) { /* alloc struct */
		printf("DEBUG: failed to initialize dynamic set struct\n");
		return NULL;
	} else if (pthread_mutexattr_init(&attr) || 
			pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE) || 
			pthread_mutex_init(&set->lock, &attr)) { /* create recursive matrix */
		printf("DEBUG: mutex init failed for set %lx\n", (int64_t)set);
		return NULL;
	} else if (!(set->data = malloc(sizeof(void *) * start_size))) { /* alloc data array */
		printf("DEBUG: failed to initialize dynamic set with size %lu\n", start_size);
		return NULL;
	}
	
	set->size = start_size;
	set->count = 0;
	printf("DEBUG: created dynamic set %lx of size %lu\n", (int64_t)set, start_size);
	return set;
}

void dyn_set_deinit(struct dyn_set *set) {
	dyn_set_lock(set);
	dyn_set_unlock(set);
	pthread_mutex_destroy(&set->lock);
	free(set->data);
	free(set);
}

inline uint8_t dyn_set_lock(struct dyn_set *set) {
	if (pthread_mutex_lock(&set->lock)) {
		printf("DEBUG: failed to lock mutex for set %lx\n", (uint64_t)set);
		return 1;
	}
	return 0;
}

inline uint8_t dyn_set_unlock(struct dyn_set *set) {
	if (pthread_mutex_unlock(&set->lock)) {
		printf("DEBUG: failed to unlock mutex for set %lx\n", (uint64_t)set);
		return 1;
	}
	return 0;
}

uint64_t dyn_set_count(struct dyn_set *set) {
	return set->count;
}

void *dyn_set_get_item(struct dyn_set *set, uint64_t index) {
	if (dyn_set_lock(set) || index >= set->count)
		return NULL;
	dyn_set_unlock(set);
	return set->data[index];
}


int8_t dyn_set_add(struct dyn_set *set, void *item) {
	if (dyn_set_lock(set) || __dyn_set_grow(set))
		return 1;

	set->data[set->count++] = item;
	printf("DEBUG: added item %lx to set %lx at index %lu\n", (uint64_t)item, (uint64_t)set, set->count - 1);
	dyn_set_unlock(set);
	return 0;
}

int8_t dyn_set_remove(struct dyn_set *set, void *item) {
	uint64_t index = __dyn_set_find(set, item);
	if (index >= set->count || dyn_set_lock(set))
		return index - set->count;
	
	set->data[index] = NULL;
	if (index < set->count - 1)
		set->data[index] = set->data[set->count - 1];
	set->count--;

	printf("DEBUG: removed item %lx from set %lx at index %lu\n", (uint64_t)item, (uint64_t)set, index);
	__dyn_set_shrink(set);
	dyn_set_unlock(set);
	return 0;
}




