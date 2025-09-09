# Python bindings for base-types

## Install

You can build this prototype with

    python setup.py build_ext -i

For now, you have to add this folder to your PYTHONPATH environment variable:

    export PYTHONPATH=$PYTHONPATH:<this folder>

You could also install the bindings to some location that is already in the
Python path:

    python setup.py install --prefix <path>

The more convenient way is to use autoproj. You must add three lines to
your manifest:

    package_sets:
       ...
       - type: git
         url: git@git.hb.dfki.de:InFuse/infuse-package_set.git

    layout:
       ...
       - tools/infuselog2cpp
       ...

## Note

There are still some open issues, e.g.
* How can we handle compressed Frame objects (e.g. JPG, PNG)?
* How can we make vectors / matricies look and feel more like numpy arrays?
* You can find todos all over the files.
* ...
