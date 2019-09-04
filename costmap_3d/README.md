Costmap 3D
==========

The `costmap_3d` package provides a 3D layered costmap based on octomaps.

The `Costmap3DROS` class specializes `Costmap2DROS` to provide smooth
backwards compatability.

Because the `Costmap3DROS` is a `Costmap2DROS`, it has a 2D costmap associated
with it, and uses the main 2D layered costmap mutex for synchronizing
the 3D costmap to keep them consistent with one another. By default, the 2D
costmap will be empty. However, there are a pair of layer plugins which can be
used to forward a flattened version of the 3D costmap to the corresponding 2D
costmap, allowing easy migration into 3D navigation. For instance, some
planners can be made to use the 3D costmap, while others continue to use the
2D costmap.

Several example costmap 3d layer plugins are provided, in addition to the
previously mentioned 3D to 2D conversion layers. An octomap server layer is
provided which subscribes to octomap updates from an octomap server. The
corresponding octomap server can integrate many different sources of data
(point clouds, laser scans), replacing the functionality in the costmap
2D obstacle layer and static layer. Also, a point cloud layer is provided which
copies the last point cloud received into a costmap layer, allowing other
data sources, such as sensor fusion layers, to integrate directly.

In addition to the 3D maps, `Costmap3DROS` provides a querying
interface in contrast to the 2D costmap. This query interface uses the
Flexible Collision Library, FCL, to perform queries against a robot mesh and
the current costmap state. Four types of queries are provided, collision,
cost, distance and signed distance.

## Caveats

The `badger-develop` branch of `navigation` is required to get the correct
API changes to `Costmap2DROS` to make it extensible (by making many
methods virtual, and by adding a few new APIs).

Cost queries are currently unimplemented, and simply return -1.0 for collision
and 0.0 otherwise.

The `badger-develop` branch of `octomap` is required for `setTreeValues`.

Certain features present in the `badger-develop` branch of `octomap_mapping`
are used by the octomap costmap layer plugin. The octomap update message
is used to make efficient map transfer possible from this branch, too.

Many important performance improvements are present in the `badger-develop`
branch of FCL to make query performance reasonable.

Signed distance queries between octomaps and meshes are not yet functional in
FCL.
