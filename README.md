#  Poirot
## Overview
## Benchmark
## Use Poirot
### Preparation
1. Install Apollo and SORA-SVL
2. It is recommended to download ***Poirot*** directly to Apollo folder
3. Before build Apollo, move folder `custom_component` into `/apollo/modules` in Apollo folder
4. Build Apollo
5. In Poirot folder, check `requirements.txt` to create satisfied environment
### How to use
1. Edit paths in `build_proto.sh` and run `sh build_proto.sh`, the results are already in folder `proto`
2. If there is a scenario similar to the benchmark's definition, you can directly use the script `test_scenario.py` in the folder `benchmark` to obtain data, including `.record.00000` and `.obj`. Or you can obtain these two using your method without such a scenario file, as long as the data format is the same.
3. ***Docker:*** In Apollo, under the path `/apollo/Poirot`, run `python msg.py --process`
4. ***Environment:*** In command line, under the path `/apollo/Poirot`, run `python gt.py`
5. ***Docker:*** In Apollo, under the path `/apollo/Poirot`, run `python msg.py --refine`
6. ***Environment:*** Run Apollo bridge and LGSVL. In command line, under the path `/apollo/Poirot`, run `python main.py`,then wait for results!
