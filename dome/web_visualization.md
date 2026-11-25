# Overview

Web visualization supports rendering and interacting with running Ignition
Gazebo simulations. You may find web visualization useful
when interacting with remote or headless Gazebo Sim simulations. No
additionally dependencies, other than a browser and internet connection, are
required.

## How to use web visualization

The following steps will guide you through the process of running Ignition
Gazebo with a websocket server and connecting to the websocket for
visualization. 

1. Start an Gazebo Sim instance as usual. We will use the `fuel.sdf`
   world and run it headless.
```
ign gazebo -v 4 -s -r fuel.sdf
```

1. Create an Ignition Launch file with the websocket server plugin.
```
echo "<?xml version='1.0'?>
<ignition version='1.0'>
  <plugin name='ignition::launch::WebsocketServer'
          filename='ignition-launch-websocket-server'>
    <port>9002</port>
  </plugin>
</ignition>" > websocket.ign
```

1. Run the websocket server using
```
ign launch -v 4 websocket.ign
```

1. Visualize the simulation by going to
   [https://app.gazebosim.org/visualization](https://app.gazebosim.org/visualization) and pressing the Connect button.

1. It may take a few seconds for the scene to load. Your browser needs to
   fetch all of the models in the world.

## Websocket server customization

The following parameters are available in the websocket server plugin.

  * `<admin_authorization_key>` : If this is set, then a connection must provide the matching key using an "auth" call on the websocket. If the `<authorization_key>` is set, then the connection can provide that key.
  * `<authorization_key>` : If this is set, then a connection must provide the
matching key using an "auth" call on the websocket. If the `<admin_authorization_key>` is set, then the connection can provide that key.
  * `<max_connections>` : An integer that is the maximum number of simultaneous connections.
  * `<port>` : An integer that is websocket port.
  * `<publication_hz>` : An integer that is the maximum publication hertz rate.

## Code and Support

1. The code for the web application,
   [app.gazebosim.org](https://app.gazebosim.org), lives at
   [https://gitlab.com/ignitionrobotics/web/app](https://gitlab.com/ignitionrobotics/web/app).

1. The javascript library used to render the 3D scene lives at
   [https://gitlab.com/ignitionrobotics/web/gz3d](https://gitlab.com/ignitionrobotics/web/gz3d).

1. Is you notice an issue with web visualization, then please
   file a ticket at
   [https://github.com/gazebo-web/gzweb/issues](https://github.com/gazebo-web/gzweb/issues).

## Troubleshooting

1. If you are running Gazebo Sim in a docker container, make sure to
   use the [--network host](https://docs.docker.com/network/network-tutorial-host/) Docker commandline option. Without `--network host` the web application won't be able to connect to the websocket server.
