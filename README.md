# Building

This project requires:

- **CMake** version ≥ 3.16  
- **C++** standard C++17  

## Compilation

```bash
cmake .
make
```

## Usage

```bash
./adRRT -m map/random-32-32-20.map -s instance/random-32-32-20-even-1.scen -o result.txt -n 100 -t 60 -r 42 -d 2
```

## Help

```bash
./adRRT --help
```

# References

Some algorithmic ideas in this implementation were inspired by the following works:

"Lacam Search-based algorithm for quick multi-agent pathfinding"

"Improved discrete RRT for coordinated multi-robot planning"

We thank the authors for their contributions, which provided valuable inspiration for the design of this project.


# License

This project is licensed under the MIT License.