# Live-Color Camera

Adapted from the Boston dynamics stitched camera example (spot-sdk/python/examples/stitch_front_images). Incorporated live color feed of spots from cameras stitched together. Rendered in a PyGame window with additional Spot status information written to screen.

Only includes two pixel formats right now. Will get mad and exit if anything else is passed in for that parameter.

1. Color - PIXEL_FORMAT_RGB_U8
2. Black and White - PIXEL_FORMAT_GREYSCALE_U8

Also should pass in a 1-100 values for jpeg quality

## To execute

live_feed_status.py

```
python live_feed_status.py [SPOT_IP] --pixel-format [PIXEL_FORMAT] -j [1-100]
```

```
python live_feed_status.py 192.168.80.3 --pixel-format PIXEL_FORMAT_RGB_U8 -j 50
```

robot_status.py

```
python robot_status.py [SPOT_IP]
```