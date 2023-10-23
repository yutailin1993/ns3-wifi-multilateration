
CO2P: A New Paradigm of Communication-Aware Collaborative Positioning for FutureG Wireless Systems
================================

## Table of Contents:

1) [An overview](#co2p-overview)
2) [Building ns-3](#building-ns-3)
3) [Running ns-3](#running-ns-3)


Note:  Much more substantial information about ns-3 can be found at
https://www.nsnam.org

## CO2P overview

This project is a simulation of our work - [A New Paradigm of Communication-Aware Collaborative Positioning for FutureG Wireless Systems](https://dl.acm.org/doi/10.1145/3565287.3610276). This work is published on ACM MobiHoc 2023. This project is based on [NS3](https://www.nsnam.org/) (ns3.36 speficifcally), and [wifi-ftm-ns3](https://github.com/tkn-tub/wifi-ftm-ns3).

There are 3 major branches in this work, i.e. centralizedCP, centralizedIP, and distributedCP which correspond to different paradigms of positioning. For more detail about the paradigms, please refer to the [paper](https://dl.acm.org/doi/10.1145/3565287.3610276).

The entry point is in `scratch/multilateration.cc` for all branches. For graph operation including constructing rigid topology for centralizedCP and MDS-based localization, please check `graph.py`.

## Building ns-3

To build the project for all branches, you can navigate to project root directory and use the following command:
```shell
./ns3 build multilateration
```

If you are switching branches, it is recommended to build the project from scratch:
```shell
./ns3 clean && ./ns3 configure && ./ns3 build multilateration
```

For the detail of build NS3 project, please refer to [NS3 document](https://www.nsnam.org/releases/ns-3-36/documentation/).

Note that this project has been tested on ubuntu 20.04 and macOS.


## Running ns-3

After done building ns3, you can run the project with following command:

```shell
./ns3 run multilateration
```
