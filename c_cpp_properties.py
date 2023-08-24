import subprocess, argparse, sys, pprint, json
from os import path

# Read the output of Chibios build to generate .vscode/c_cpp_properties.json
# E.g. make | python c_cpp_properties.py -C arm-none-eabi-g++ > .vscode/c_cpp_properties.json

parser = argparse.ArgumentParser()
parser.add_argument('-C', '--compiler', dest='compiler', default='arm-none-eabi-gcc')
parser.add_argument('-x', '--language', dest='language', default='c++')
parser.add_argument('--configuration', dest='configuration', default='Windows')
parser.add_argument('--intellisense-mode', dest='intelliSenseMode', default='clang-x64')

cc_parser = argparse.ArgumentParser()
cc_parser.add_argument('-I', dest='includePath', action='append', default=[])
cc_parser.add_argument('-D', dest='defines', action='append')

def main(args = None):
    if args is None:
        args = sys.argv

    args = parser.parse_args(args[1:])
    includePath, defines = compilerIncludePath(args)
    cincludePath, cdefines = parseCommandLine(args)

    includePath += [x for x in cincludePath if not (x in includePath)]
    defines += [x for x in cdefines if not (x in defines)]

    json.dump({ 
        'configurations': [
            {
                "name": args.configuration,
                "includePath": includePath,
                "browse": {
                    "limitSymbolsToIncludedHeaders": True,
                    "databaseFilename": "",
                    "path": includePath
                },
                "defines": defines,
                "intelliSenseMode": "clang-x64"
            }
        ],
        "version": 3
    },
    sys.stdout,
    indent=4)

def compilerIncludePath(args):
    compiler = subprocess.Popen(
        [args.compiler, '-mcpu=cortex-m4', '-mthumb', '-DCORTEX_USE_FPU=FALSE', '-E', '-P', '-v', '-dD', '-'], 
        stdin = subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    includeSection = False
    includePath = []
    defines = []
    out, err = compiler.communicate(input='')

    for line in bytes.decode(err).splitlines():
        if includeSection and line.startswith(' '):
            includePath.append(path.abspath(line.strip()))
        elif line.startswith('#include'):
            includeSection = True
        elif line.startswith('End'):
            includeSection = False

    for line in bytes.decode(out).splitlines():
        if line.startswith('#define'):
            parts = line.split(' ', maxsplit=2)
            if len(parts) == 3:
                item = parts[1] + '=' + parts[2]
                if item not in defines:
                    defines.append(item)
    
    return includePath, defines

    while not next(line).starts_with('#include'):
        pass

def parseCommandLine(args):
    includePath = []
    defines = []

    for line in sys.stdin.readlines():
        cc_args = line.split()
        if len(cc_args) and cc_args[0] == args.compiler:
            cc_args, unknown =  cc_parser.parse_known_args(cc_args[1:])
            for item in cc_args.includePath:
                if item not in includePath:
                    includePath.append(item)
            if cc_args.defines:
                for item in cc_args.defines:
                    if item not in defines:
                        defines.append(item)
    return includePath, defines

if __name__ == "__main__":
    main()