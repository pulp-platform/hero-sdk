/*
 * This file is part of the PULP device driver.
 *
 * Copyright (C) 2018 ETH Zurich, University of Bologna
 *
 * Author: Pirmin Vogel <vogelpi@iis.ee.ethz.ch>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef _PULP_MEM_H_
#define _PULP_MEM_H_

#include <linux/module.h>    /* Needed by all modules */
#include <linux/kernel.h>    /* KERN_ALERT, container_of */
#include <linux/slab.h>      /* kmalloc */
#include <linux/mm.h>        /* vm_area_struct struct, page struct, PAGE_SHIFT, page_to_phys */
#include <linux/highmem.h>   /* kmap, kunmap */

#include "pulp_module.h"

// methods
void pulp_mem_cache_flush(struct page * page, unsigned offset_start, unsigned offset_end);
void pulp_mem_cache_inv(struct page * page, unsigned offset_start, unsigned offset_end);
unsigned  pulp_mem_get_num_pages(unsigned addr_start, unsigned size_b);
int pulp_mem_get_user_pages(struct page *** pages, unsigned addr_start, unsigned n_pages, unsigned write);
int pulp_mem_map_sg(unsigned ** addr_start_vec, unsigned ** addr_end_vec, unsigned long ** addr_offset_vec,
                    unsigned ** page_start_idxs, unsigned ** page_end_idxs, 
                    struct page *** pages, unsigned n_pages, 
                    unsigned addr_start, unsigned addr_end);
int pulp_mem_check_num_sg( struct page *** pages, unsigned n_pages);
int pulp_mem_l2_get_entries(unsigned ** virtual_page_vec, unsigned ** phy_page_vec,
                            struct page *** pages, unsigned n_pages, unsigned addr_start);

#endif/*_PULP_MEM_H_*/
