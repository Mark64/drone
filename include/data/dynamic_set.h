// dynamically sized set holding any data type
// by Mark Hill
#ifndef __dynamic_set_h
#define __dynamic_set_h

// placeholder; defined in implementation
struct dyn_set;

/*
 * initializes a dynamic set with a start size of start_size
 * must be freed with dyn_set_deinit()
 * @return		a pointer to the set
 * 				NULL if an error occured creating the set or start_size is 0
 */
struct dyn_set *dyn_set_init(long long unsigned int start_size);

/*
 * cleans the dynamic set and frees all associated memory
 * does not free the data pointed to by set members themselves,
 * 	just the set container
 */
void dyn_set_deinit(struct dyn_set *set);

/*
 * @return		the number of items in the underlying set
 * 				0 if set is NULL
 */
long long unsigned int dyn_set_count(struct dyn_set *set);

/*
 * @return		a void * array with all the items in the set
 * 				NULL if set is NULL
 */
void **dyn_set_items(struct dyn_set *set);

/* 
 * adds the specified item to the end of the set pointed to by set
 * @return		0 on success
 * 				1 on memory allocation error
 * 				2 if set or item is NULL
 */
char dyn_set_add(struct dyn_set *set, void *item);

/*
 * removes the specified item, if it exists, from set
 * @return		0 on success
 * 				1 on item not in set
 * 				2 if set or item is NULL
 */
char dyn_set_remove(struct dyn_set *set, void *item);

#endif
