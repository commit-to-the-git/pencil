# Pencil CLI Usage

Pencil is a command-line program. This document explains every flag and
shows copy-paste-ready examples for Windows and Linux.

## TL;DR, quick examples

```bash
# Render an example scene (writes out a .ppm image):
pencil examples/three_spheres.pecl

# Render the built-in demo:
pencil --demo

# Override the output file:
pencil examples/three_spheres.pecl my_render.ppm

# Render bigger and cleaner:
pencil examples/three_spheres.pecl --width 1280 --height 960 --samples 8

# Render with BVH disabled (slower, but useful for benchmarking):
pencil examples/bvh_stress.pecl --no-bvh
```

On Windows, the binary is named `pencil.exe`. All flags are identical,
just substitute `pencil.exe` for `pencil` in any of the examples below.

---

## How to run `pencil.exe` on Windows

1. Download `pencil.exe` (or `pencil.exe.bin`, rename it to `pencil.exe`).
2. Open a terminal: `Win+R` → type `cmd` → Enter. Or open PowerShell.
3. `cd` to the folder where you saved `pencil.exe`.
4. Run it:

   ```cmd
   pencil.exe examples\three_spheres.pecl
   ```

   You'll get a file named `three_spheres.ppm` in the current folder.

5. To view a `.ppm` image, you can use any of:
   - IrfanView (free, lightweight)
   - GIMP
   - ImageMagick: `magick three_spheres.ppm three_spheres.png`
   - Online converters (search "ppm to png")

You can also use Pencil on Windows from PowerShell exactly the same way.

---

## How to run `pencil` on Linux / macOS

1. Make sure you have the binary in your path or in the current directory.
2. If you only have the source code, build it:
   ```bash
   g++ -O2 -std=c++17 src/pencil.cpp -o pencil -lpthread
   ```
3. Render a scene:
   ```bash
   ./pencil examples/three_spheres.pecl
   ```
4. View the result:
   ```bash
   # ImageMagick:
   display three_spheres.ppm
   # or convert:
   magick three_spheres.ppm three_spheres.png
   # feh:
   feh three_spheres.ppm
   ```

---

## Full command reference

```
pencil <scene.pecl> [output.ppm] [options]
pencil --demo [output.ppm] [options]
pencil --help
```

### Positional arguments

| Argument       | Required? | Description |
|----------------|-----------|-------------|
| `<scene.pecl>` | Yes (or `--demo`) | Path to a `.pecl` scene file. |
| `[output.ppm]` | No | Output file path. Defaults to the scene name with `.ppm`. |

### Flags

| Flag                | Argument | Default | Description |
|---------------------|----------|---------|-------------|
| `--width N`         | integer  | from scene | Override output width in pixels. |
| `--height N`        | integer  | from scene | Override output height in pixels. |
| `--samples N`       | integer  | from scene | Override AA samples per pixel (actual = N×N jittered). |
| `--depth N`         | integer  | from scene | Override max reflection/refraction recursion depth. |
| `--threads N`       | integer  | auto   | Override worker thread count (default: hardware concurrency). |
| `--no-bvh`          | (none)   | BVH on  | Disable BVH, use brute-force linear intersection. |
| `--benchmark`       | (none)   | off     | Run scene twice (BVH + no-BVH), print stats, no output file. |
| `--quiet`           | (none)   | off     | Don't print progress info. |
| `--demo`            | (none)   | off     | Render the built-in demo scene (ignores `<scene>`). |
| `--version`, `-v`   | (none)   | off     | Print version (`v0.0.1 beta`) and exit. |
| `--help`, `-h`      | (none)   | off     | Show help and exit. |

CLI flags **override** the values inside the `.pecl` file's `render { }`
block. So you can use a scene file as a "starting point" and tweak size /
samples / depth on the command line.

---

## What gets printed

By default, Pencil prints a small summary before rendering:

```
pencil v0.0.1 beta - cpu ray tracer
  scene:   examples/three_spheres.pecl
  output:  three_spheres.ppm
  size:    640 x 480
  aa:      4x4 spp (16 samples/pixel)
  depth:   8
  lights:  2
  prims:   4
  bvh:     on
  threads: 8
rendered in 1.423 s (1,310,720 rays, 0.92 Mrays/s)
wrote three_spheres.ppm
```

Use `--quiet` to silence this (useful for batch rendering).

---

## Examples (copy-paste ready)

### 1. Render the built-in demo

```bash
pencil --demo
# -> writes pencil_demo.ppm
```

```bash
pencil --demo demo.ppm --width 800 --height 600
# -> writes demo.ppm at 800x600
```

### 2. Render an example scene

```bash
pencil examples/three_spheres.pecl
# -> writes three_spheres.ppm in the current directory
```

### 3. Render at a specific size

```bash
pencil examples/glass.pecl out.ppm --width 1920 --height 1080
# -> 1080p render of glass.pecl
```

### 4. Render high-quality (more AA samples)

```bash
pencil examples/cornell_box.pecl high.ppm --samples 16
# -> 16x16 = 256 samples per pixel (very clean, slower)
```

### 5. Reduce quality for fast iteration

```bash
pencil examples/surf.pecl preview.ppm --samples 1 --width 320 --height 240
# -> tiny preview render
```

### 6. Disable BVH for benchmarking

```bash
# Time the BVH version:
time pencil examples/bvh_stress.pecl bvh.ppm --quiet

# Time the brute-force version:
time pencil examples/bvh_stress.pecl nobvh.ppm --no-bvh --quiet
```

### 7. Run the built-in benchmark

```bash
pencil --benchmark examples/bvh_stress.pecl
```

Output looks like:

```
================ benchmark results ================
  BVH:       0.239 s
  No BVH:    3.021 s
  Speedup:   12.7x
  Rays:      683,771 (BVH)
             683,771 (no BVH)
  Mrays/s:   2.86 (BVH)
             0.23 (no BVH)
  Threads:   2
  Primitives:1001
====================================================
```

### 8. Render an OBJ mesh

```bash
pencil examples/mesh_test.pecl
```

Loads `examples/models/diablo3_pose.obj` (about 5000 triangles) and renders
it. Use the `mesh { }` block in your `.pecl` to load any OBJ:

```
mat { clr: <75, 75, 80> }
mesh {
  file:  examples/models/african_head.obj
  scale: 1.5
  pos:   <0, 0, 0>
}
```

### 9. Batch-render every scene

```bash
# Linux/macOS:
for f in examples/*.pecl; do
  pencil "$f" --quiet
done

# Windows CMD:
for %f in (examples\*.pecl) do pencil.exe "%f" --quiet

# Windows PowerShell:
Get-ChildItem examples\*.pecl | ForEach-Object { pencil.exe $_.FullName --quiet }
```

### 10. Re-render all tests into the tests/ folder

Pencil ships with a helper script:

```bash
# Linux/macOS:
bash tests/render_all.sh
```

This re-renders every example scene and the demo, saving PNGs into `tests/`.
Requires Python 3 + Pillow for the PPM to PNG conversion step. If Pillow is
not installed, the script leaves `.ppm` files in place.

---

## Writing your own `.pecl` scene

A `.pecl` file is a plain text file describing a 3D scene in a deliberately
simple block-based format. See `README.md` for the full grammar, or copy
`examples/three_spheres.pecl` as a starting template.

Minimal example, a single red sphere on a gray background:

```
bg: <10, 10, 10>
amb: <10, 10, 10>

cmra {
  pos: <0, 0, -5>
  aim: <0, 0, 0>
  up:  <0, 1, 0>
  fov: 60
}

mat { clr: <100, 0, 0> }
sphere { pos: <0, 0, 0>, rad: 1 }

light {
  type: point
  pos:  <2, 3, -2>
  clr:  <100, 100, 100>
}

render {
  size: 400 400
  spp: 2
  depth: 4
}
```

Save this as `myscene.pecl` and render it:

```bash
pencil myscene.pecl
```

You'll get `myscene.ppm`, a 400×400 image of a red sphere.

---

## Converting PPM to PNG

Pencil writes PPM because it requires zero dependencies. To convert to PNG:

```bash
# ImageMagick (cross-platform):
magick input.ppm output.png

# ffmpeg:
ffmpeg -i input.ppm output.png

# Python + Pillow:
python3 -c "from PIL import Image; Image.open('input.ppm').save('output.png')"

# GIMP: open the .ppm, then File → Export As → output.png
```

---

## Exit codes

| Code | Meaning                              |
|------|--------------------------------------|
| 0    | Success                              |
| 2    | Unknown argument on the command line |
| 3    | Scene file could not be parsed       |
| 4    | Output file could not be written     |

---

## Troubleshooting

**"Cannot open scene file"**, the path you gave doesn't exist or isn't
readable. Try an absolute path: `pencil /full/path/to/scene.pecl`.

**"expected key, got ','"**, the parser is fairly strict about syntax.
Make sure every `key: value` pair has a colon, and the only commas are
inside `<x, y, z>` vectors or between `key: value` pairs.

**Image is all black**, check that your scene has at least one `light { }`
block and that the camera `aim` is pointing at the geometry.

**Render is very slow**, try reducing `--samples`, `--depth`, or
`--width`/`--height`. Make sure BVH is on (it's on by default; if you see
`BVH: off (linear)` in the output, you forgot to remove `--no-bvh`).

**`.exe` won't run on Windows**, Pencil is a 64-bit binary. It needs a
64-bit Windows. If you see "not a valid Win32 application", you're trying
to run it on 32-bit Windows (very rare these days).

**Windows Defender flags the .exe**, false positives are common with
statically-linked MinGW binaries. You may need to right-click → Properties
→ Unblock, or add an exclusion. The source code is provided so you can
verify safety or rebuild from scratch:

```bash
x86_64-w64-mingw32-g++ -O2 -std=c++17 -static src/pencil.cpp -o pencil.exe
```
