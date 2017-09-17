// dynamic set implementation
// by Mark Hill

#include<stdlib.h>
#include<stdio.h>

#include<dynamic_set.h>


struct dyn_set {
	void **data;
	long long int size;
	long long int count;
};

char __dyn_set_resize(struct dyn_set *set) {
	void **temp;
	if ((temp = realloc(set->data, sizeof(void *) * (set->size * 2)))) {
		set->data = temp;
		printf("DEBUG: resized array %llx from size %llu to %llu\n", (long long unsigned int)set->data, set->size, set->size * 2);
		return 0;
	} else {
		printf("DEBUG: error resizing array %llx from size %llu to %llu\n", (long long unsigned int)set->data, set->size, set->size * 2);
		return 1;
	}
}

long long unsigned int __dyn_set_find(struct dyn_set *set, void *item) {
	if (set == NULL || set->data == NULL || item == NULL) {
		printf("DEBUG: NULL item, set, or set->data in ___dyn_set_find()\n");
		return set->count + 1;
	}

	for (long long int i = 0; i < set->count; i++) {
		if (set->data[i] == item) {
			printf("DEBUG: found item %llx at index %llu\n", (long long unsigned int)item, i);
			return i;
		}
	}

	printf("DEBUG: could not find item %llx in set %llx\n", (long long unsigned)item, (long long unsigned)set);
	return set->count;
}



struct dyn_set *dyn_set_init(long long unsigned int start_size) {
	if (start_size <= 1) {
		printf("DEBUG: requested 0 size dynamic set - returning NULL\n");
		return NULL;
	}

	struct dyn_set *set = malloc(sizeof(struct dyn_set));
	if (set) {
		set->data = malloc(sizeof(void *) * start_size);
		if (set->data) {
			set->size = start_size;
			set->count = 0;
			printf("DEBUG: created dynamic set %llx of size %llu\n", (long long int)set, start_size);
			return set;
		}
	}
	printf("DEBUG: failed to initialize dynamic set with size %llu\n", start_size);
	return NULL;
}

void dyn_set_deinit(struct dyn_set *set) {
	free(set->data);
	free(set);
}


long long unsigned int dyn_set_count(struct dyn_set *set) {
	return set->count;
}

void **dyn_set_items(struct dyn_set *set) {
	return set->data;
}


char dyn_set_add(struct dyn_set *set, void *item) {
	if (set->count == set->size && __dyn_set_resize(set))
		return 1;
	set->data[set->count++] = item;
	printf("DEBUG: added item %llx to set %llx at index %llu\n", (long long unsigned int)item, (long long unsigned int)set, set->count - 1);
	return 0;
}

char dyn_set_remove(struct dyn_set *set, void *item) {
	long long unsigned int index = __dyn_set_find(set, item);
	if (index == set->count)
		return 1;
	else if (index == set->count + 1)
		return 2;
	
	set->data[index] = NULL;
	if (index < set->count - 1)
		set->data[index] = set->data[set->count - 1];
	set->count--;
	printf("DEBUG: removed item %llx from set %llx at index %llu\n", (long long unsigned int)item, (long long unsigned int)set, index);
	return 0;
}




