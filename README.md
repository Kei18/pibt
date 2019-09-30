Simulator of Iterative Multi-agent Path Finding
===
Let multiple agents move smoothly.
See [the project page](https://kei18.github.io/pibt).

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
- Scenario: https://www.movingai.com/benchmarks/mapf/index.html

## Author
[Keisuke Okumura](https://github.com/Kei18)
