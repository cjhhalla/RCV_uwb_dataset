# [Addressing Relative Pose Impact on UWB Localization: Dataset Introduction and Analysis]
This site presents the UWB datasets collected from drone platform
* The six sequence of drone path
* TWR based UWB ranging measurements with four anchors.
* Stereo VIO setup equiped in drone
* We capture drone's and UWB ground truth pose by using Qualisys motion capture


# Usage

- **UWB ROS Message**: Described in the `uwb` folder.
- **Dataset Folder**: Contains `anchor_n.csv` files for each sequence.
- **Ground Truth Anchor Pose**: Located in `anchor.yaml` (x, y, z, roll, pitch, yaw).
- **Ground Truth Pose**: According to flying in `gt.csv`.


# Downloads
You can download full rosbag file on Drive
<a name="tab-download"></a>
<table class="tg">
<thead>
  <tr>
    <th class="tg-6ibf">Dataset</th>
    <th class="tg-6ibf">Link</th>
    <th class="tg-6ibf">Size</th>
    <th class="tg-6ibf">Duration</th>
    <th class="tg-6ibf">Remark</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td class="tg-v8dz">Sequence1</td>
    <td class="tg-6ibf"><a href="https://drive.google.com/file/d/1Jhw7XsIYu2Vrwtn08faMl7vYwbdgdcYL/view?usp=drive_link" target="_blank" rel="noopener noreferrer">.zip</a></td>
    <td class="tg-6ibf">41.3 MB</td>
    <td class="tg-6ibf">209s</td>
    <td class="tg-v8dz">Azimuth angle & Elevation angle </td>
  </tr>
  <tr>
    <td class="tg-v8dz">Sequence2</td>
    <td class="tg-9m02"><a href="https://drive.google.com/file/d/1F4t2GqY-9ezIc4wEYbCeMUADx1SmnoJx/view?usp=drive_link" target="_blank" rel="noopener noreferrer">.zip</a></td>
    <td class="tg-6ibf">60.8 MB</td>
    <td class="tg-6ibf">291 s</td>
    <td class="tg-v8dz">Azimuth angle & Range</td>
  </tr>
  <tr>
    <td class="tg-v8dz">Sequence3</td>
    <td class="tg-9m02"><a href="https://drive.google.com/file/d/1fApwS1YDeotqCV5BU-KjjOvlU3H-Ahw3/view?usp=drive_link" target="_blank" rel="noopener noreferrer">.zip</a></td>
    <td class="tg-6ibf">xx GB</td>
    <td class="tg-6ibf">xx s</td>
    <td class="tg-v8dz">not yet!</td>
  </tr>
  <tr>
    <td class="tg-v8dz">Sequence4</td>
    <td class="tg-9m02"><a href="https://drive.google.com/file/d/1fApwS1YDeotqCV5BU-KjjOvlU3H-Ahw3/view?usp=drive_link" target="_blank" rel="noopener noreferrer">.zip</a></td>
    <td class="tg-6ibf">xx GB</td>
    <td class="tg-6ibf">xx s</td>
    <td class="tg-v8dz">not yet!</td>
  </tr>
  <tr>
    <td class="tg-v8dz">Sequence5</td>
    <td class="tg-9m02"><a href="https://drive.google.com/file/d/1fApwS1YDeotqCV5BU-KjjOvlU3H-Ahw3/view?usp=drive_link" target="_blank" rel="noopener noreferrer">.zip</a></td>
    <td class="tg-6ibf">73.1 MB</td>
    <td class="tg-6ibf">309 s</td>
    <td class="tg-v8dz">Range</td>
  </tr>
  <tr>
    <td class="tg-v8dz">Sequence6</td>
    <td class="tg-9m02"><a href="https://drive.google.com/file/d/1fApwS1YDeotqCV5BU-KjjOvlU3H-Ahw3/view?usp=drive_link" target="_blank" rel="noopener noreferrer">.zip</a></td>
    <td class="tg-6ibf">xx GB</td>
    <td class="tg-6ibf">xx s</td>
    <td class="tg-v8dz">Complex path</td>
  </tr>
  </tr>
</tbody>
</table>




# Updates

**06/12/2024**: UWB dataset with precise anchor and tag relative pose without stereo VIO setup data

  # Citation
If you use some resource from this data suite, please cite it as

```
@article{cjh2024RCVuwb,
  title   = {Addressing Relative Pose Impact on UWB Localization: Dataset Introduction and Analysis},
  author  = {Jun Hyeok Choe and Inwook Shim},
}
```
# Credits
Thanks to hardware developer JJ Kang (jeonjo0727@gmail.com) at Inha University.
