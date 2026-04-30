# trk-blogs

Source code accompanying each blog post on
[talharehmanmtrkt.github.io/TalhaRehmanMTRKT](https://talharehmanmtrkt.github.io/TalhaRehmanMTRKT/).

```
trk-blogs/
└── p<id>/    # one folder per post; the id matches p<id>.tex
    ├── ...   # any source files (.cpp, .py, .m, ...)
    └── *.csv # any tabular data referenced from the post
```

Source files contain marker comments of the form

```
// snippet#N lines NNN-NNN : <short label>
```

so the website's HTML generator can extract the right block to render
inside the post.

## Posts

| id | title                                                                                  |
|----|----------------------------------------------------------------------------------------|
| p1 | Introduction to Optimization with C++/CPLEX: A Microgrid Dispatch Example              |
| p2 | From Single Energy Microgrid to Multi Energy: Adding Heat to the Microgrid Dispatch    |
| p3 | Adding Electric Vehicles as Flexible Storage to a Multi-Energy Microgrid               |
| p4 | Closing the Loop with Cooling: A Three-Carrier Microgrid Dispatch                      |
