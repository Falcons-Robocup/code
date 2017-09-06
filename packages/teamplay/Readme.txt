[alias]
        ps = push --recurse-submodules=on-demand
        pl = !git pull && git submodule update --init --recursive --remote --merge

^ Add the above aliases to your config to recursively pull and push the submodule

Push with:
git ps

Pull with:
git pl
