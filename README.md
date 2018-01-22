# The NTHU Global Router ISPD2008

This library/application is the work of the following authors : Yen-Jung Chang, Yu-Ting Lee, Tsung-Hsien Lee, Jhih-Rong Gao, Pei-Ci Wu, and Ting-Chi Wang. 
Contact: NTHU.Route@gmail.com
[homepage of the project.](http://www.cs.nthu.edu.tw/~tcwang/nthuroute/)



## Overview
NTHU-Route 2.0 is a fast and stable global router for VLSI design. It improves the solution quality and runtime of NTHU-Route by the following enhancements: (1) a new history based cost function, (2) new ordering methods for congested region identification and rip-up and reroute, and (3) two implementation oriented techniques. At the ISPD 2008 Global Routing Contest, NTHU-Route 2.0 generated the best solutions for 11 of 16 benchmarks among all participating global routers and won the 1st place. 

For the present release on github, I (Florian Prud'homme) did some maintenance work to adapt the source code for modern C++ compilers. As a side effect, I enhance the memory management (no more malloc or explicit new, delete or free). No more segfault ! I rely on Boost library for most of the data structure. And I hope the code is more easy to read now. 

As far as I know, this the most complete open source Global router for VLSI design in the world. I hope this project will enable people to re-use the present data structures and algorithms in order to speed up innovation and research.

## Usage

### How to compile

For the moment, I recommend you to import the project on eclipse CDT, in order to compile it with the option you want. Otherwise, you can untar the archive `nthuRoute3.tar.gz`, and then type `./make` inside the uncompressed directory.

### As a Library

You have to include `#include <src/router/Route.h>`
You just fill the Input object NTHUR::RoutingRegion following the explanations at [ ISPD 2008 Global Routing Contest website](http://archive.sigda.org/ispd2008/contests/ispd08rc.html)

```C++
 NTHUR::Route router;
        NTHUR::RoutingRegion rr(3, 3, 2);
        rr.setVerticalCapacity(1, 2);
        rr.setHorizontalCapacity(0, 2);
        rr.setNetNumber(1);
        rr.beginAddANet("A", 0, 2, 1);
        rr.addPin(0, 0, 1);
        rr.addPin(2, 0, 1);
        rr.endAddANet();
        rr.adjustEdgeCapacity(1, 0, 0, 2, 0, 0, 0);
        rr.adjustEdgeCapacity(1, 1, 0, 2, 1, 0, 0);

        rr.adjustEdgeCapacity(0, 0, 1, 0, 1, 1, 0);
        rr.adjustEdgeCapacity(1, 1, 1, 1, 2, 1, 0);

        NTHUR::OutputGeneration output(router.process(rr,spdlog::level::trace));

        NTHUR::OutputGeneration::Comb comb(output.combAllNet());

        cr->scale(40., 40.);
        cr->translate(10,10);

        for (const std::vector<NTHUR::Segment3d>& v : comb) {
            for (const NTHUR::Segment3d& s: v) {
                cr->move_to( s.first.x , s.first.y );
                cr->line_to( s.last .x , s.last.y );
            }
        }
        cr->scale(1./40., 1./40.);
        cr->set_line_width(1);
        cr->stroke();
```

### Usage as an application
``` bash
NthuRoute --input=adaptec1.capo70.3d.35.50.90.gr --output=output --p2-max-iteration=150 --p2-init-box-size=25 --p2-box-expand-size=1 --overflow-threshold=00 --p3-max-iteration=20 --p3-init-box-size=10 --p3-box-expand-size=15 --monotonic-routing=0
```

#### Description of the options :
```
./route --input=testcase_file_name --output=output_file_name [options] 

Options.

--p2-init-box-size=number
Initial bounding-box size in Adaptive Multi-source Multi-sink Maze Routing in the main stage

--p2-box-expand-size=number 
Bounding-box expanding size in Adaptive Multi-source Multi-sink Maze Routing in the main stage

--p2-max-iteration=number
Maximum number of iterations in the main stage

--p3-init-box-size=number
Initial bounding-box size in Adaptive Multi-source Multi-sink Maze Routing in the refinement stage

--p3-box-expand-size=number 
Bounding-box expanding size in Adaptive Multi-source Multi-sink Maze Routing in the refinement stage

--p3-max-iteration=number
Maximum number of iterations in the refinement stage

--overflow-threshold=number
Overflow threshold in the main stage

--monotonic-routing={1,0} 
Enable/disable monotonic in each routing iteration 
```

## Related publications
T.-H. Lee and T.-C. Wang, “Robust Layer Assignment for Via Optimization in Multi-layer Global Routing,” in Proceedings of International Symposium on Physical Design (ISPD), San Diego, California, USA, March 2009, pp. 159-166. [link](http://vlsicdb.cs.nthu.edu.tw/~lab221/nthuroute/Layer_Assignment.pdf)

Yen-Jung Chang, Yu-Ting Lee, and Ting-Chi Wang, “NTHU-Route 2.0: A Fast and Stable Global Router,” in Proceedings of International Conference on Computer-Aided Design (ICCAD), San Jose, CA, USA, November 2008, pp. 338-343 [link](http://vlsicdb.cs.nthu.edu.tw/~lab221/nthuroute/NTHU-Route2.0.pdf)


Tsung-Hsien Lee and Ting-Chi Wang, “Congestion-Constrained Layer Assignment for Via Minimization in Global Routing,” IEEE Transactions on Computer-Adided Design of Integrated Circuits and Systems, September 2008, pp. 1643-1656 [link](http://vlsicdb.cs.nthu.edu.tw/~lab221/nthuroute/04603083.pdf)


Jhih-Rong Gao, Pei-Ci Wu, and Ting-Chi Wang, “A New Global Router for Modern Designs,” in Proceedings of Asia and South Pacific Design Automation Conference, Seoul, Korea, 2008, pp. 232–237 [link](http://vlsicdb.cs.nthu.edu.tw/~lab221/nthuroute/NTHU-Route.pdf)

## Contributing
I hope you will find this project useful. I now there is a lot of improvement to do (documentation, optimization, adding alternative algorithms etc...). I'm looking for people who can work on this project. I would like to work more on it, but I have to find funding for that. If you need some support and you have money, you can contact me !

