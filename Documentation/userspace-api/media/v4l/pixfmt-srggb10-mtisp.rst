.. SPDX-License-Identifier: GFDL-1.1-no-invariants-or-later
.. c:namespace:: V4L

.. _v4l2-pix-fmt-mtisp-sbggr10:
.. _v4l2-pix-fmt-mtisp-sgbrg10:
.. _v4l2-pix-fmt-mtisp-sgrbg10:
.. _v4l2-pix-fmt-mtisp-srggb10:
.. _v4l2-pix-fmt-mtisp-y10:

***********************************************************************************************************************************************************************************
V4L2_PIX_FMT_MTISP_SBGGR10 ('MBBA'), V4L2_PIX_FMT_MTISP_SGBRG10 ('MBGA'), V4L2_PIX_FMT_MTISP_SGRBG10 ('MBgA'), V4L2_PIX_FMT_MTISP_SRGGB10 ('MBRA'), V4L2_PIX_FMT_MTISP_Y10 ('MBYA')
***********************************************************************************************************************************************************************************

========================================================
10-bit Bayer and greyscale formats, MediaTek ISP packing
========================================================

Description
===========

These four pixel formats are written and read by the camera blocks of
MediaTek SoCs (CAMSV, ISP pass 1 and pass 2). They are raw sRGB / Bayer
formats with 10 bits per sample. Each line is one little endian bit
stream: sample n occupies bits 10n to 10n + 9 of the line, so four samples
take five bytes. The number of bytes per line is at least
ceil(width * 10 / 8); the driver reports it in bytesperline, bytes behind
the last sample of a line are padding.

In other respects the Bayer formats are similar to :ref:`V4L2-PIX-FMT-SRGGB10`.

V4L2_PIX_FMT_MTISP_Y10 has the same packing with samples of a single
component, as written for a 10-bit greyscale bus format
(MEDIA_BUS_FMT_Y10_1X10). Sensors send phase detection pixels this way, on
a virtual channel of their own; the arrangement of the samples in the frame
is specific to the sensor.
Below is the first group of four samples of a line in
V4L2_PIX_FMT_MTISP_SBGGR10 format.

**Byte Order.**
Each cell is one byte.

.. flat-table::

    * - start + 0:
      - B\ :sub:`00low`
      - G\ :sub:`01low`\ (bits 7--2)

        B\ :sub:`00high`\ (bits 1--0)
      - B\ :sub:`02low`\ (bits 7--4)

        G\ :sub:`01high`\ (bits 3--0)
      - G\ :sub:`03low`\ (bits 7--6)

        B\ :sub:`02high`\ (bits 5--0)
      - G\ :sub:`03high`
