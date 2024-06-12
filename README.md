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
<style type="text/css">
.tg  {border-collapse:collapse;border-spacing:0;}
.tg td{border-color:black;border-style:solid;border-width:1px;font-family:Arial, sans-serif;font-size:14px;
  overflow:hidden;padding:10px 5px;word-break:normal;}
.tg th{border-color:black;border-style:solid;border-width:1px;font-family:Arial, sans-serif;font-size:14px;
  font-weight:normal;overflow:hidden;padding:10px 5px;word-break:normal;}
.tg .tg-6ibf{border-color:inherit;font-size:18px;text-align:center;vertical-align:top}
.tg .tg-v8dz{border-color:inherit;font-size:18px;text-align:left;vertical-align:top}
.tg .tg-9m02{border-color:inherit;color:#00E;font-size:18px;text-align:center;text-decoration:underline;vertical-align:top}
</style>
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
    <td class="tg-9m02"><a href="https://researchdata.ntu.edu.sg/api/access/datafile/68132" target="_blank" rel="noopener noreferrer">.zip</a></td>
    <td class="tg-6ibf">4.3 GB</td>
    <td class="tg-6ibf">181.4 s</td>
    <td class="tg-v8dz">not yet!</td>
  </tr>
  <tr>
    <td class="tg-v8dz">Sequence4</td>
    <td class="tg-9m02"><a href="https://researchdata.ntu.edu.sg/api/access/datafile/68144" target="_blank" rel="noopener noreferrer">.zip</a></td>
    <td class="tg-6ibf">8.6 GB</td>
    <td class="tg-6ibf">396.3 s</td>
    <td class="tg-v8dz">not yet!</td>
  </tr>
  <tr>
    <td class="tg-v8dz">Sequence5</td>
    <td class="tg-9m02"><a href="https://drive.google.com/file/d/1fApwS1YDeotqCV5BU-KjjOvlU3H-Ahw3/view?usp=drive_link" target="_blank" rel="noopener noreferrer">.zip</a></td>
    <td class="tg-6ibf">73.1GB</td>
    <td class="tg-6ibf">309 s</td>
    <td class="tg-v8dz">Range</td>
  </tr>
  <tr>
    <td class="tg-v8dz">Sequence6</td>
    <td class="tg-9m02"><a href="https://researchdata.ntu.edu.sg/api/access/datafile/68142" target="_blank" rel="noopener noreferrer">.zip</a></td>
    <td class="tg-6ibf">9.0 GB</td>
    <td class="tg-6ibf">411.2 s</td>
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
