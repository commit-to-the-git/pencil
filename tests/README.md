# pencil test renders

every png in this folder was rendered by pencil itself from the corresponding
.pecl scene file in ../examples/. these are reference images, use them
to verify your build of pencil still produces the expected output.

## re-rendering all tests

```bash
# from the pencil/ project root:
bash tests/render_all.sh
```

this script invokes ./pencil on every examples/*.pecl and writes pngs
into this folder. requires python+PIL to convert PPM to PNG (if you don't
have PIL, you'll just get .ppm files which are still valid).

## image list

| image                  | scene                      | notes                                              |
|------------------------|----------------------------|----------------------------------------------------|
| demo.png               | (built-in --demo)          | 3 spheres + plane + 2 lights, the classic showcase  |
| three_spheres.png      | three_spheres.pecl         | diffuse + glass + mirror spheres                   |
| cornell_box.png        | cornell_box.pecl           | Cornell-style box with emissive panel              |
| pyramid.png            | pyramid.pecl               | 4-triangle pyramid under directional sun           |
| glass.png              | glass.pecl                 | two refractive glass spheres, depth-10             |
| surf.png               | surf.pecl                  | implicit surfaces: sphere, torus, metaball         |
| shapes.png             | shapes.pecl                | sphere, cube, cylinder, cone, mirror sphere        |
| mesh_test.png          | mesh_test.pecl             | diablo3_pose.obj mesh (~5000 tris) loaded via mesh{}|
| mesh_bench.png         | mesh_bench.pecl            | smaller african_head.obj mesh, for benchmarking    |
| bvh_stress.png         | bvh_stress.pecl            | 1000-sphere grid with BVH (about 0.24s render)     |
| bvh_stress_nobvh.png   | bvh_stress.pecl --no-bvh   | same scene without BVH (about 3.0s render, 13x slower) |

## performance summary

benchmark results from --benchmark flag, on a 2-core machine:

| scene              | prims | bvh    | linear  | speedup |
|--------------------|-------|--------|---------|---------|
| three_spheres      |     5 | 1.4s   | 1.4s    | 1.0x    |
| bvh_stress         |  1001 | 0.24s  | 3.02s   | 12.7x   |
| mesh_bench (head)  |  2492 | 0.06s  | 4.87s   | 79.7x   |

the bvh builds once at scene-load time and is reused for both primary rays
and shadow rays. multithreading splits the image into horizontal strips,
one per worker thread.
