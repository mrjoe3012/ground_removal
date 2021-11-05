# How to Build

Make sure you have cmake and libpcl installed.

```bash
sudo apt install cmake libpcl-dev
```

Navigate into the root directory and run

```bash
cmake -S ./ -B build/
```
then to build, run

```bash
bash build.bash
```

## How to visualize

*Note that there is a collection of .pcd files under the directory 'sample_data/'.*

### Basic

After building, run

```bash
build/vis <.pcd file here> basic
```

To run a basic visualization.

### Bins/Segments

```bash
build/vis <.pcd file here> bins 1 <number of bins here>
```

```bash
build/vis <.pcd file here> segments <number of segments here> 1
```

### Ground Plane Lines

```bash
build/vis <.pcd file here> lines <number of segments here> <number of bins here>
```

### Ground Removal Before and After

```bash
build/vis <.pcd file here> beforeafter <number of segments here> <number of bins here>
```
