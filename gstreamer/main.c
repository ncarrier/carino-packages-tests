#include <stdlib.h>
#include <stdio.h>

#include <gst/gst.h>

#define PIPELINE "v4l2src name=source device=%s ! fakesink"

int main(int argc, char *argv[])
{
	int status = EXIT_FAILURE;
	char *pipeline_desc;
	GstElement *pipeline = NULL;
	GError *err = NULL;
	GstStateChangeReturn ret;
	GstMessage *msg;
	GstBus *bus;
	const char *video_device;
	GstElement *src;
	GstPad *pad = NULL;
	char *name;
	GstCaps *caps = NULL;

	gst_init(&argc, &argv);

	if (argc != 2) {
		fprintf(stderr, "usage %s device\n", argv[0]);
		goto out;
	}
	video_device = argv[1];

	pipeline_desc = g_strdup_printf(PIPELINE, video_device);
	if (pipeline_desc == NULL) {
		fprintf(stderr, "allocation error\n");
		goto out;
	}
	pipeline = gst_parse_launch(pipeline_desc, &err);
	if (pipeline == NULL || err != NULL)
		goto out;
	/* Start the pipeline and wait for max. 10 seconds for it to start up */
	gst_element_set_state(pipeline, GST_STATE_PLAYING);
	ret = gst_element_get_state(pipeline, NULL, NULL,
			10 * GST_SECOND);

	/* Check if any error messages were posted on the bus */
	bus = gst_element_get_bus(pipeline);
	msg = gst_bus_poll(bus, GST_MESSAGE_ERROR, 0);
	gst_object_unref(bus);


	if (msg != NULL || ret != GST_STATE_CHANGE_SUCCESS)
		goto out;

	gst_element_set_state(pipeline, GST_STATE_PAUSED);

	src = gst_bin_get_by_name(GST_BIN(pipeline), "source");

	g_object_get(G_OBJECT(src), "device-name", &name, NULL);
	if (name == NULL)
		name = "Unknown";

	g_print("Device: %s (%s)\n", name, video_device);
	pad = gst_element_get_static_pad(src, "src");
	caps = gst_pad_query_caps(pad, NULL);
	g_print("Caps: %s\n", gst_caps_to_string(caps));
	gst_element_set_state(pipeline, GST_STATE_NULL);

	status = EXIT_SUCCESS;
out:
	if (caps != NULL)
		gst_caps_unref(caps);
	if (pad != NULL)
		gst_object_unref(pad);
	if (pipeline != NULL)
		gst_object_unref(pipeline);
	if (err != NULL)
		g_free(err);
	g_free(pipeline_desc);

	return status;
}
