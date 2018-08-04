/* Reset the USB */
{
  fsbl_printf(DEBUG_GENERAL, "Reset USB...\r\n");
		  
  /* Set data dir */
  *(unsigned int *)0xe000a284 = 0x00000001;
		  
  /* Set OEN */
  *(unsigned int *)0xe000a288 = 0x00000001;
  Xil_DCacheFlush();
  /* For REVB Set data value low for reset, then back high */
#ifdef ZED_REV_A
  *(unsigned int *)0xe000a048 = 0x00000001;
  Xil_DCacheFlush();
  *(unsigned int *)0xe000a048 = 0x00000000;
  Xil_DCacheFlush();
#else
  *(unsigned int *)0xe000a048 = 0x00000000;
  Xil_DCacheFlush();
  *(unsigned int *)0xe000a048 = 0x00000001;
  Xil_DCacheFlush();
#endif
}
