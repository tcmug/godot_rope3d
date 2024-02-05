# Godot Rope 3D WIP

A gdextension version of the Godot 3.x rope implementation https://github.com/tcmug/rope3d in to Godot 4.x

Stay tuned.

## Development

### Setup

Clone this repository & run the following commands:

```
git submodule update --init
scons ./godot-cpp
```

The latter will build the required libs and the include files.

### Compiling
```
scons
```
Or if you want to target a platform (linux, macos, windows, android, ios, javascript):
```
scons platform=macos
```

