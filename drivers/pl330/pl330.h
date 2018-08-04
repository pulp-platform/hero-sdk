/*
 * The contents of this file are extensions to Linux DMA API.
 * This file should eventually be moved to public include path.
 */
#ifndef _DRIVERS_DMA_PL330_H_
#define _DRIVERS_DMA_PL330_H_

/**
 * enum pl330_ctrl_cmd - PL330 externsions to dma_ctrl_cmd. DMA operations
 * that can optionally be exercised on a running channel.
 * @PL330_SLAVE_CONFIG: this command is only implemented by PL330 DMAC
 * that need to runtime reconfigure the slave channels (as opposed to passing
 * configuration data in statically from the platform). An additional
 * argument of struct pl330_slave_config must be passed in with this
 * command.
 */
enum pl330_ctrl_cmd {
	PL330_SLAVE_CONFIG = 0x10 /* Must be higher than dma_ctrl_cmd enum. */
};

/**
 * struct pl330_slave_config - pl330 dma slave channel runtime config
 * @flush_once:
 *    false: flush on every word
 *    true: flush once at the beginning of DMA request
 * @wfp_once:
 *    false: WFP on every word
 *    true: WFP once at the beginning of DMA request
 * @unroll_loop:
 *    false: Do not unroll loop. Make loops small as possible.
 *    true: Unroll loops into as many repeated loop bodies that can fit.
 */
struct pl330_slave_config {
	bool flush_once;
	bool wfp_once;
	bool unroll_loop;
};

#endif/*_DRIVERS_DMA_PL330_H_*/

