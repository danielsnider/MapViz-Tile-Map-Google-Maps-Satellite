# MapViz Tile Map + Google Maps Satellite +  MapProxy
This will walk you through using [MapProxy](https://mapproxy.org/) in a docker container to proxy Google Maps satellite view into a [WMTS](https://en.wikipedia.org/wiki/Web_Map_Tile_Service) tile service so that it can be viewed by ROS's [MapViz Tile Map plugin](https://github.com/swri-robotics/mapviz#tile-map).


## Dependencies

- Docker
- ROS
- MapViz and with following plugins: 

```
sudo apt-get install ros-kinetic-mapviz ros-kinetic-mapviz-plugins ros-kinetic-tile-map
```

## Setup


### 1. Create MapProxy server

1.1. Create the MapProxy configuration file and folder. The cached map tiles will be written to `~/mapproxy/cache_data`.

```
mkdir ~/mapproxy
cp ./mapproxy.yaml ~/mapproxy/mapproxy.yaml
```

The `mapproxy.yaml` file configures MapProxy to serve Google Maps satellite view over WMTS protocol.

1.2. Start the MapProxy server with `~/mapproxy` as a shared volume. 

```
docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
```

1.3. Confirm MapProxy is working by browsing to [http://127.0.0.1:8080/demo/](http://127.0.0.1:8080/demo/). You will see the MapProxy logo and if you click on "Image-format png" you will get an interactive map in your browser. You can also see the first map tile by browsing to [http://localhost:8080/wmts/gm_layer/gm_grid/0/0/0.png](http://localhost:8080/wmts/gm_layer/gm_grid/0/0/0.png).

### 2. Setup MapViz

2.1. Open MapViz using ROS

```
roslaunch mapviz mapviz.launch
```

2.2. Click the "Add" button. 

2.3. Choose to add a new `map_tile` display component.

2.4. In the "Source" dropdown select "Custom WMTS Source...".

2.5. In the "Base URL:" field enter the following:

```
http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png
```

2.6. In the "Max Zoom:" field enter `19`

2.7. Click "Save..."

Congrats! You should now see Google Maps load in MapViz.


## FAQ
### Where are MapProxy cache files?

```
~/mapproxy/cache_data
```

### How to set default MapViz position?

```
$ vim ~/.mapviz_config 
# edit the following
offset_x: 1181506
offset_y: -992564.2
```

OR using ROS parameters:

```
$ roscd mapviz
$ vim launch/mapviz.launch
<launch>
    <node pkg="tf" type="static_transform_publisher" name="swri_transform" args="0 0 0 0 0 0 /map /check 100"  />
    <node pkg="swri_transform_util" type="initialize_origin.py" name="initialize_origin" >
        <param name="local_xy_frame" value="/map"/>
        <param name="local_xy_origin" value="swri"/>
        <rosparam param="local_xy_origins">
            [{ name: swri,
             latitude: 37.9879772, <!-- change this -->
             longitude: 23.9078602, <!-- change this -->
             altitude: 129.69,
             heading: 0.0}]
        </rosparam>
    </node>
    <node pkg="mapviz" type="mapviz" name="mapviz"/>
</launch>
```

### How to publish GPS coordinates over ROS

```
rostopic pub /novatel/fix sensor_msgs/NavSatFix "{latitude: 38.406222, longitude: -110.792027}"
```

### Note for corporate users

In 2013 there was [some discussion](http://gis.stackexchange.com/questions/56982/how-to-use-mapproxy-to-serve-wms-from-reprojected-google-maps-tiles) about Google's Terms & Conditions stating that you are only allowed to access the tiles through Google's API (ie. not MapProxy). 
