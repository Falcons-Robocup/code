# RtDB v3

Third Generation of CAMBADA's Real-time Database

http://robotica.ua.pt/CAMBADA/

Modified by Falcons: 
- upgrading comm to comm2 (and later renaming it back to comm)
- redesign database backend
- remove legacy rtdb1 adapter
- extend semaphore-based wait functionality
- rewrite basic interfacing
- TODO: include logging library


## Requirements

This project uses some 3rd-party libraries:
- LZ4
- zstd
- LMDB
- Msgpack

## Instructions

```
cd build
cmake ..
make
```

## Also included

- Comm: process used to broadcast data between agents in the team
- Two compressors available (Lz4, zstd)
- Tool: dictionary generator (to update the dictionary for compression)
- Tool: rtop (provides realtime info of all items in RtDB) and many more command-line utilities

## License

RtDB is licensed under GNU General Public License v3 (GPL-3)

