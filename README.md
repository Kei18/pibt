PIBT
===
Let multiple agents move smoothly.
See [the project page](https://kei18.github.io/pibt).

__IMPORTANT NOTES__
- In v1.0-v1.2, the PPS implementation includes bugs. Use the latest (≥v1.3).
You can check the PPS results [here](/others/pps-latest-result.md).
The results are not so different from the PIBT paper.
- [A new version (pibt2)](https://kei18.github.io/pibt2), substantially updated for a journal paper, is out.
  *I strongly recommend using the new one instead of this repo.*

## Demo
Multi-agent Path Finding

![MAPF](/docs/images/mapf.gif)

Multi-agent Pickup and Delivery (sushi-mode)

![MAPD](/docs/images/sushi.gif)

## Requirement
The visualization relies on [openFrameworks](https://openframeworks.cc).
You need to install openFrameworks beforehand and export `OF_ROOT` of your environment.
```
export OF_ROOT={your openFrameworks directory}
```

The latest implementation relies on [boost](https://www.boost.org/).

## Usage
At first, you must prepare param file.
To confirm details, see [a sample file](sample-param.txt).
Then you can execute the simulator as following.

- implementation with openFrameworks
```
make of
make ofrun param=sample-param.txt
```

- without visualization (for experiment)
```
make c
make crun param=sample-param.txt
```

## Licence
This software is released under the MIT License, see [LICENSE.txt](LICENCE.txt).

## Others
- Maps: https://www.movingai.com/benchmarks/grids.html
- Images: https://www.irasutoya.com
- Font: https://fonts.google.com/
- Scenario: https://www.movingai.com/benchmarks/mapf/index.html

## Author
[Keisuke Okumura](https://kei18.github.io) is currently a Ph.D. candidate at Tokyo Institute of Technology, working on multiple moving agents.
