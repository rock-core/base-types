from distutils.core import setup
from distutils.extension import Extension
from distutils.sysconfig import get_config_vars
from Cython.Build import cythonize
import numpy
import os


def strict_prototypes_workaround():
    # Workaround to remove '-Wstrict-prototypes' from compiler invocation
    opt = get_config_vars('OPT')[0]
    os.environ['OPT'] = " ".join(flag for flag in opt.split()
                                 if flag != '-Wstrict-prototypes')


if __name__ == '__main__':
    strict_prototypes_workaround()

    extensions = [
        Extension(
            "basetypes",
            [
                "basetypes.pyx"
            ],
            include_dirs=[
                ".",
                numpy.get_include(),
                os.environ.get("AUTOPROJ_CURRENT_ROOT") + "/install/include/",
                "/usr/include/eigen3/"  # TODO
            ],
            library_dirs=[
                os.environ.get("AUTOPROJ_CURRENT_ROOT") + "/install/lib/"
            ],
            libraries=[
                "base-types"
            ],
            define_macros=[
                ("NDEBUG",),
             ],
            extra_compile_args=[
                "-std=c++11",
                "-O3",
                # disable warnings caused by Cython using the deprecated
                # NumPy C-API
                "-Wno-cpp", "-Wno-unused-function"
            ],
            language="c++"
        )
    ]
    setup(
        name="basetypes",
        ext_modules=cythonize(extensions),
        description="Python bindings for Rock base-types",
        version="0.0.1",
        maintainer="Alexander Fabisch",
        maintainer_email="Alexander.Fabisch@dfki.de",
        packages=[""],
        package_dir={"": "."},
        package_data={
            "": ["_basetypes.pxd", "basetypes.pxd", "basetypes.pyx"]
        }
    )
