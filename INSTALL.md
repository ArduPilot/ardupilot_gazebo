# Install

## Ubtunu 18.04

Gazebo11 is required.

```bash
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt-get install libgazebo11
$ sudo apt-get install libgazebo11-dev
$ sudo apt-get install gazebo11
```

If you are seeing errors like:

```bash
$ gazebo --verbose
Gazebo multi-robot simulator, version 11.3.0
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org

[Msg] Waiting for master.
Gazebo multi-robot simulator, version 11.3.0
Copyright (C) 2012 Open Source Robotics Foundation.
Released under the Apache 2 License.
http://gazebosim.org

[Msg] Waiting for master.
[Msg] Connected to gazebo master @ http://127.0.0.1:11345
[Msg] Publicized address: 192.168.1.170
[Err] [RTShaderSystem.cc:478] Unable to find shader lib. Shader generating will fail.[Wrn] [SystemPaths.cc:459] File or path does not exist [""] [worlds/empty.world]
[Err] [Server.cc:444] Could not open file[worlds/empty.world]
[Wrn] [Server.cc:359] Falling back on worlds/empty.world
[Wrn] [SystemPaths.cc:459] File or path does not exist [""] [worlds/empty.world]
[Err] [Server.cc:444] Could not open file[worlds/empty.world]
[Msg] Connected to gazebo master @ http://127.0.0.1:11345
[Msg] Publicized address: 192.168.1.170
[Err] [RTShaderSystem.cc:478] Unable to find shader lib. Shader generating will fail.
```

You may need to update your `~/.bashrc`:

```bash
# Gazebo
- source /usr/share/gazebo-9/setup.sh
+ source /usr/share/gazebo-11/setup.sh
```

RapidJSON is required:

```
$ sudo apt-get update
$ sudo apt-get install rapidjson-dev
```
