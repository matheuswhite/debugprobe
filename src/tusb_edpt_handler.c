/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tusb_edpt_handler.h"
#include "DAP.h"

#define WR_IDX(x) (x.wptr % DAP_PACKET_COUNT)
#define RD_IDX(x) (x.rptr % DAP_PACKET_COUNT)

#define WR_SLOT_PTR(x) &(x.data[WR_IDX(x)][0])
#define RD_SLOT_PTR(x) &(x.data[RD_IDX(x)][0])

static uint8_t itf_num;
static uint8_t _rhport;

static uint8_t _out_ep_addr;
static uint8_t _in_ep_addr;

static buffer_t USBRequestBuffer;
static buffer_t USBResponseBuffer;

static uint8_t DAPRequestBuffer[DAP_PACKET_SIZE];
static uint8_t DAPResponseBuffer[DAP_PACKET_SIZE];

volatile uint32_t _resp_len;

bool buffer_full(buffer_t *buffer)
{
	return ((buffer->wptr + 1) % DAP_PACKET_COUNT == buffer->rptr);
}

bool buffer_empty(buffer_t *buffer)
{
	return (buffer->wptr == buffer->rptr);
}

void dap_edpt_init(void)
{
}

void dap_edpt_reset(uint8_t __unused rhport)
{
	itf_num = 0;
}

char *dap_cmd_string[] = {
	[ID_DAP_Info] = "DAP_Info",
	[ID_DAP_HostStatus] = "DAP_HostStatus",
	[ID_DAP_Connect] = "DAP_Connect",
	[ID_DAP_Disconnect] = "DAP_Disconnect",
	[ID_DAP_TransferConfigure] = "DAP_TransferConfigure",
	[ID_DAP_Transfer] = "DAP_Transfer",
	[ID_DAP_TransferBlock] = "DAP_TransferBlock",
	[ID_DAP_TransferAbort] = "DAP_TransferAbort",
	[ID_DAP_WriteABORT] = "DAP_WriteABORT",
	[ID_DAP_Delay] = "DAP_Delay",
	[ID_DAP_ResetTarget] = "DAP_ResetTarget",
	[ID_DAP_SWJ_Pins] = "DAP_SWJ_Pins",
	[ID_DAP_SWJ_Clock] = "DAP_SWJ_Clock",
	[ID_DAP_SWJ_Sequence] = "DAP_SWJ_Sequence",
	[ID_DAP_SWD_Configure] = "DAP_SWD_Configure",
	[ID_DAP_SWD_Sequence] = "DAP_SWD_Sequence",
	[ID_DAP_JTAG_Sequence] = "DAP_JTAG_Sequence",
	[ID_DAP_JTAG_Configure] = "DAP_JTAG_Configure",
	[ID_DAP_JTAG_IDCODE] = "DAP_JTAG_IDCODE",
	[ID_DAP_SWO_Transport] = "DAP_SWO_Transport",
	[ID_DAP_SWO_Mode] = "DAP_SWO_Mode",
	[ID_DAP_SWO_Baudrate] = "DAP_SWO_Baudrate",
	[ID_DAP_SWO_Control] = "DAP_SWO_Control",
	[ID_DAP_SWO_Status] = "DAP_SWO_Status",
	[ID_DAP_SWO_ExtendedStatus] = "DAP_SWO_ExtendedStatus",
	[ID_DAP_SWO_Data] = "DAP_SWO_Data",
	[ID_DAP_QueueCommands] = "DAP_QueueCommands",
	[ID_DAP_ExecuteCommands] = "DAP_ExecuteCommands",
};

uint16_t dap_edpt_open(uint8_t __unused rhport, tusb_desc_interface_t const *itf_desc,
		       uint16_t max_len)
{
	tusb_desc_endpoint_t *edpt_desc;
	uint8_t ep_addr;

	TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == itf_desc->bInterfaceClass &&
			  DAP_INTERFACE_SUBCLASS == itf_desc->bInterfaceSubClass &&
			  DAP_INTERFACE_PROTOCOL == itf_desc->bInterfaceProtocol,
		  0);

	/* Initialise circular buffer indices */
	USBResponseBuffer.wptr = 0;
	USBResponseBuffer.rptr = 0;
	USBRequestBuffer.wptr = 0;
	USBRequestBuffer.rptr = 0;

	/* Initialse full/empty flags */
	USBResponseBuffer.wasFull = false;
	USBResponseBuffer.wasEmpty = true;
	USBRequestBuffer.wasFull = false;
	USBRequestBuffer.wasEmpty = true;

	uint16_t const drv_len = sizeof(tusb_desc_interface_t) +
				 (itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t));
	TU_VERIFY(max_len >= drv_len, 0);
	itf_num = itf_desc->bInterfaceNumber;

	/* Initialising the OUT endpoint */
	edpt_desc = (tusb_desc_endpoint_t *)(itf_desc + 1);
	ep_addr = edpt_desc->bEndpointAddress;

	_out_ep_addr = ep_addr;

	/* The OUT endpoint requires a call to usbd_edpt_xfer to initialise the endpoint, giving
	 * tinyUSB a buffer to consume when a transfer occurs at the endpoint */
	usbd_edpt_open(rhport, edpt_desc);
	usbd_edpt_xfer(rhport, ep_addr, WR_SLOT_PTR(USBRequestBuffer), DAP_PACKET_SIZE);

	/* Initiliasing the IN endpoint */

	edpt_desc += 1;
	ep_addr = edpt_desc->bEndpointAddress;

	_in_ep_addr = ep_addr;

	/* The IN endpoint doesn't need a transfer to initialise it, as this will be done by the
	 * main loop of dap_thread */
	usbd_edpt_open(rhport, edpt_desc);

	return drv_len;
}

bool dap_edpt_control_xfer_cb(uint8_t __unused rhport, uint8_t stage,
			      tusb_control_request_t const *request)
{
	return false;
}

/* Manage USBResponseBuffer (request) write and USBRequestBuffer (response) read indices */
bool dap_edpt_xfer_cb(uint8_t __unused rhport, uint8_t ep_addr, xfer_result_t result,
		      uint32_t xferred_bytes)
{
	(void)result;

	const uint8_t ep_dir = tu_edpt_dir(ep_addr);

	switch (ep_dir) {
	case TUSB_DIR_IN:
		if (xferred_bytes > DAP_PACKET_SIZE) {
			return false;
		}

		USBResponseBuffer.rptr++;

		/* This checks that the buffer was not empty in DAP thread, which means the
		 * next buffer was not queued up for the in endpoint callback So, queue up
		 * the buffer at the new read index, since we expect read to catch up to
		 * write at this point. It is possible for the read index to be multiple
		 * spaces behind the write index (if the USB callbacks are lagging behind
		 * dap thread), so we account for this by only setting wasEmpty to true if
		 * the next callback will empty the buffer
		 */
		if (!USBResponseBuffer.wasEmpty) {
			usbd_edpt_xfer(rhport, ep_addr, RD_SLOT_PTR(USBResponseBuffer),
				       (uint16_t)_resp_len);
			USBResponseBuffer.wasEmpty =
				(USBResponseBuffer.rptr + 1) == USBResponseBuffer.wptr;
		}

		/* Wake up DAP thread after processing the callback */
		vTaskResume(dap_taskhandle);

		return true;
	case TUSB_DIR_OUT:
		if (xferred_bytes > DAP_PACKET_SIZE) {
			return false;
		}

		/* Only queue the next buffer in the out callback if the buffer is not full
		 * If full, we set the wasFull flag, which will be checked by dap thread
		 */
		if (!buffer_full(&USBRequestBuffer)) {
			USBRequestBuffer.wptr += 1;
			usbd_edpt_xfer(rhport, ep_addr, WR_SLOT_PTR(USBRequestBuffer),
				       DAP_PACKET_SIZE);
			USBRequestBuffer.wasFull = false;
		} else {
			USBRequestBuffer.wasFull = true;
		}

		/* Wake up DAP thread after processing the callback */
		vTaskResume(dap_taskhandle);

		return true;
	default:
		return false;
	}
}

_Noreturn void dap_thread(void *ptr)
{
	(void)ptr;

	uint32_t n;

	while (true) {

		while (USBRequestBuffer.rptr != USBRequestBuffer.wptr) {
			/*
			 * Atomic command support - buffer QueueCommands, but don't process them
			 * until a non-QueueCommands packet is seen.
			 */
			n = USBRequestBuffer.rptr;
			while (USBRequestBuffer.data[n % DAP_PACKET_COUNT][0] ==
			       ID_DAP_QueueCommands) {
				probe_info("%u %u DAP queued cmd %s len %02x\n",
					   USBRequestBuffer.wptr, USBRequestBuffer.rptr,
					   dap_cmd_string[USBRequestBuffer
								  .data[n % DAP_PACKET_COUNT][0]],
					   USBRequestBuffer.data[n % DAP_PACKET_COUNT][1]);

				USBRequestBuffer.data[n % DAP_PACKET_COUNT][0] =
					ID_DAP_ExecuteCommands;
				n += 1;

				while (n == USBRequestBuffer.wptr) {
					/* Need yield in a loop here, as IN callbacks will also wake
					 * the thread */
					probe_info("DAP wait\n");
					vTaskSuspend(dap_taskhandle);
				}
			}

			/* Read a single packet from the USB buffer into the DAP Request buffer */
			memcpy(DAPRequestBuffer, RD_SLOT_PTR(USBRequestBuffer), DAP_PACKET_SIZE);

			probe_info("%u %u DAP cmd %s len %02x\n", USBRequestBuffer.wptr,
				   USBRequestBuffer.rptr, dap_cmd_string[DAPRequestBuffer[0]],
				   DAPRequestBuffer[1]);

			USBRequestBuffer.rptr += 1;

			/* If the buffer was full in the out callback, we need to queue up another
			 * buffer for the endpoint to consume, now that we know there is space in
			 * the buffer. */
			if (USBRequestBuffer.wasFull) {
				/* Suspend the scheduler to safely update the write index */
				vTaskSuspendAll();

				USBRequestBuffer.wptr += 1;
				usbd_edpt_xfer(_rhport, _out_ep_addr, WR_SLOT_PTR(USBRequestBuffer),
					       DAP_PACKET_SIZE);
				USBRequestBuffer.wasFull = false;

				xTaskResumeAll();
			}

			_resp_len = DAP_ExecuteCommand(DAPRequestBuffer, DAPResponseBuffer);

			probe_info("%u %u DAP resp %s\n", USBResponseBuffer.wptr,
				   USBResponseBuffer.rptr, dap_cmd_string[DAPResponseBuffer[0]]);

			/* Suspend the scheduler to avoid stale values/race conditions between
			 * threads */
			vTaskSuspendAll();

			const bool is_buffer_empty = buffer_empty(&USBResponseBuffer);
			memcpy(WR_SLOT_PTR(USBResponseBuffer), DAPResponseBuffer,
			       (uint16_t)_resp_len);
			USBResponseBuffer.wptr += 1;

			if (is_buffer_empty) {
				usbd_edpt_xfer(_rhport, _in_ep_addr, RD_SLOT_PTR(USBResponseBuffer),
					       (uint16_t)_resp_len);
			} else {
				/* The In callback needs to check this flag to know when to queue up
				 * the next buffer. */
				USBResponseBuffer.wasEmpty = false;
			}

			xTaskResumeAll();
		}

		/* Suspend DAP thread until it is awoken by a USB thread callback */
		vTaskSuspend(dap_taskhandle);
	}
}

usbd_class_driver_t const _dap_edpt_driver = {
	.init = dap_edpt_init,
	.reset = dap_edpt_reset,
	.open = dap_edpt_open,
	.control_xfer_cb = dap_edpt_control_xfer_cb,
	.xfer_cb = dap_edpt_xfer_cb,
	.sof = NULL,
#if CFG_TUSB_DEBUG >= 2
	.name = "DAP ENDPOINT",
#endif
};

/* Add the custom driver to the tinyUSB stack */
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
	*driver_count = 1;
	return &_dap_edpt_driver;
}
