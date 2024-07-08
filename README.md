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
- **Analysis Tool**: in analysis_tool directory

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
    <td class="tg-6ibf"><a href="https://drive.google.com/file/d/1R7qCP1irliEQNGPcMXcCt5ZlD9UFQ_ni/view?usp=drive_link" target="_blank" rel="noopener noreferrer">.zip</a></td>
    <td class="tg-6ibf">41.3 MB</td>
    <td class="tg-6ibf">209s</td>
    <td class="tg-v8dz">Azimuth angle & Elevation angle </td>
  </tr>
  <tr>
    <td class="tg-v8dz">Sequence2</td>
    <td class="tg-9m02"><a href="https://drive.google.com/file/d/1FWZj5fSxbY9b3wyzM7ZuNIv5jtfZ7wbU/view?usp=drive_link" target="_blank" rel="noopener noreferrer">.zip</a></td>
    <td class="tg-6ibf">60.8 MB</td>
    <td class="tg-6ibf">291 s</td>
    <td class="tg-v8dz">Azimuth angle & Range</td>
  </tr>
  <tr>
    <td class="tg-v8dz">Sequence3</td>
    <td class="tg-9m02"><a href="https://drive.google.com/file/d/10UPBtVhZdw5lrLnUnOWTzvp7IuZAYogS/view?usp=drive_link" target="_blank" rel="noopener noreferrer">.zip</a></td>
    <td class="tg-6ibf">47.3 MB</td>
    <td class="tg-6ibf">253s</td>
    <td class="tg-v8dz">Difference heading anlge</td>
  </tr>
  <tr>
    <td class="tg-v8dz">Sequence4</td>
    <td class="tg-9m02"><a href="https://drive.google.com/file/d/11SCxuJnXOu_iU_3EFOQ7MfGxKKJhzqFy/view?usp=drive_link" target="_blank" rel="noopener noreferrer">.zip</a></td>
    <td class="tg-6ibf">42.0 MB</td>
    <td class="tg-6ibf">191 s</td>
    <td class="tg-v8dz">Elevation anlge</td>
  </tr>
  <tr>
    <td class="tg-v8dz">Sequence5</td>
    <td class="tg-9m02"><a href="https://drive.google.com/file/d/11eUDDW0q20CxYyxKkIWn0v1lnfOyWH03/view?usp=drive_link" target="_blank" rel="noopener noreferrer">.zip</a></td>
    <td class="tg-6ibf">73.1 MB</td>
    <td class="tg-6ibf">309 s</td>
    <td class="tg-v8dz">Range</td>
  </tr>
  <tr>
    <td class="tg-v8dz">Sequence6</td>
    <td class="tg-9m02"><a href="https://drive.google.com/file/d/18QkmtvjA0mgVV1Id3eOfSvZvn4547OiB/view?usp=drive_link" target="_blank" rel="noopener noreferrer">.zip</a></td>
    <td class="tg-6ibf">30.0 MB</td>
    <td class="tg-6ibf">154 s</td>
    <td class="tg-v8dz">Complex path, Azimuth & Elevation angle</td>
  </tr>
  </tr>
</tbody>
</table>




# Updates

**06/12/2024**: UWB dataset with precise anchor and tag relative pose without stereo VIO setup data

  # Citation
Paper is in [this url](https://arxiv.org/abs/2407.03890) If you use some resource from this data suite, please cite it as

```
@article{cjh2024RCVuwb,
  title   = {Addressing Relative Pose Impact on UWB Localization: Dataset Introduction and Analysis},
  author  = {Jun Hyeok Choe and Inwook Shim},
  journal = {arXiv}
  year = {2024}
}
```
# Credits
Thanks to hardware developer JJ Kang (jeonjo0727@gmail.com) at Inha University.
