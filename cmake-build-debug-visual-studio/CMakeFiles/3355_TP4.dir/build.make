# CMAKE generated file: DO NOT EDIT!
# Generated by "NMake Makefiles" Generator, CMake Version 3.15

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

!IF "$(OS)" == "Windows_NT"
NULL=
!ELSE
NULL=nul
!ENDIF
SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\JetBrains\CLion 2019.3.5\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\JetBrains\CLion 2019.3.5\bin\cmake\win\bin\cmake.exe" -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\cmake-build-debug-visual-studio"

# Include any dependencies generated for this target.
include CMakeFiles\3355_TP4.dir\depend.make

# Include the progress variables for this target.
include CMakeFiles\3355_TP4.dir\progress.make

# Include the compile flags for this target's objects.
include CMakeFiles\3355_TP4.dir\flags.make

CMakeFiles\3355_TP4.dir\main.cpp.obj: CMakeFiles\3355_TP4.dir\flags.make
CMakeFiles\3355_TP4.dir\main.cpp.obj: ..\main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\cmake-build-debug-visual-studio\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/3355_TP4.dir/main.cpp.obj"
	C:\PROGRA~2\MICROS~1\2019\COMMUN~1\VC\Tools\MSVC\1423~1.281\bin\Hostx86\x86\cl.exe @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) /FoCMakeFiles\3355_TP4.dir\main.cpp.obj /FdCMakeFiles\3355_TP4.dir\ /FS -c "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\main.cpp"
<<

CMakeFiles\3355_TP4.dir\main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/3355_TP4.dir/main.cpp.i"
	C:\PROGRA~2\MICROS~1\2019\COMMUN~1\VC\Tools\MSVC\1423~1.281\bin\Hostx86\x86\cl.exe > CMakeFiles\3355_TP4.dir\main.cpp.i @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\main.cpp"
<<

CMakeFiles\3355_TP4.dir\main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/3355_TP4.dir/main.cpp.s"
	C:\PROGRA~2\MICROS~1\2019\COMMUN~1\VC\Tools\MSVC\1423~1.281\bin\Hostx86\x86\cl.exe @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) /FoNUL /FAs /FaCMakeFiles\3355_TP4.dir\main.cpp.s /c "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\main.cpp"
<<

CMakeFiles\3355_TP4.dir\object.cpp.obj: CMakeFiles\3355_TP4.dir\flags.make
CMakeFiles\3355_TP4.dir\object.cpp.obj: ..\object.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\cmake-build-debug-visual-studio\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/3355_TP4.dir/object.cpp.obj"
	C:\PROGRA~2\MICROS~1\2019\COMMUN~1\VC\Tools\MSVC\1423~1.281\bin\Hostx86\x86\cl.exe @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) /FoCMakeFiles\3355_TP4.dir\object.cpp.obj /FdCMakeFiles\3355_TP4.dir\ /FS -c "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\object.cpp"
<<

CMakeFiles\3355_TP4.dir\object.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/3355_TP4.dir/object.cpp.i"
	C:\PROGRA~2\MICROS~1\2019\COMMUN~1\VC\Tools\MSVC\1423~1.281\bin\Hostx86\x86\cl.exe > CMakeFiles\3355_TP4.dir\object.cpp.i @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\object.cpp"
<<

CMakeFiles\3355_TP4.dir\object.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/3355_TP4.dir/object.cpp.s"
	C:\PROGRA~2\MICROS~1\2019\COMMUN~1\VC\Tools\MSVC\1423~1.281\bin\Hostx86\x86\cl.exe @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) /FoNUL /FAs /FaCMakeFiles\3355_TP4.dir\object.cpp.s /c "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\object.cpp"
<<

CMakeFiles\3355_TP4.dir\parser.cpp.obj: CMakeFiles\3355_TP4.dir\flags.make
CMakeFiles\3355_TP4.dir\parser.cpp.obj: ..\parser.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\cmake-build-debug-visual-studio\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/3355_TP4.dir/parser.cpp.obj"
	C:\PROGRA~2\MICROS~1\2019\COMMUN~1\VC\Tools\MSVC\1423~1.281\bin\Hostx86\x86\cl.exe @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) /FoCMakeFiles\3355_TP4.dir\parser.cpp.obj /FdCMakeFiles\3355_TP4.dir\ /FS -c "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\parser.cpp"
<<

CMakeFiles\3355_TP4.dir\parser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/3355_TP4.dir/parser.cpp.i"
	C:\PROGRA~2\MICROS~1\2019\COMMUN~1\VC\Tools\MSVC\1423~1.281\bin\Hostx86\x86\cl.exe > CMakeFiles\3355_TP4.dir\parser.cpp.i @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\parser.cpp"
<<

CMakeFiles\3355_TP4.dir\parser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/3355_TP4.dir/parser.cpp.s"
	C:\PROGRA~2\MICROS~1\2019\COMMUN~1\VC\Tools\MSVC\1423~1.281\bin\Hostx86\x86\cl.exe @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) /FoNUL /FAs /FaCMakeFiles\3355_TP4.dir\parser.cpp.s /c "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\parser.cpp"
<<

CMakeFiles\3355_TP4.dir\raytracer.cpp.obj: CMakeFiles\3355_TP4.dir\flags.make
CMakeFiles\3355_TP4.dir\raytracer.cpp.obj: ..\raytracer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\cmake-build-debug-visual-studio\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/3355_TP4.dir/raytracer.cpp.obj"
	C:\PROGRA~2\MICROS~1\2019\COMMUN~1\VC\Tools\MSVC\1423~1.281\bin\Hostx86\x86\cl.exe @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) /FoCMakeFiles\3355_TP4.dir\raytracer.cpp.obj /FdCMakeFiles\3355_TP4.dir\ /FS -c "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\raytracer.cpp"
<<

CMakeFiles\3355_TP4.dir\raytracer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/3355_TP4.dir/raytracer.cpp.i"
	C:\PROGRA~2\MICROS~1\2019\COMMUN~1\VC\Tools\MSVC\1423~1.281\bin\Hostx86\x86\cl.exe > CMakeFiles\3355_TP4.dir\raytracer.cpp.i @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\raytracer.cpp"
<<

CMakeFiles\3355_TP4.dir\raytracer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/3355_TP4.dir/raytracer.cpp.s"
	C:\PROGRA~2\MICROS~1\2019\COMMUN~1\VC\Tools\MSVC\1423~1.281\bin\Hostx86\x86\cl.exe @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) /FoNUL /FAs /FaCMakeFiles\3355_TP4.dir\raytracer.cpp.s /c "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\raytracer.cpp"
<<

CMakeFiles\3355_TP4.dir\extras.cpp.obj: CMakeFiles\3355_TP4.dir\flags.make
CMakeFiles\3355_TP4.dir\extras.cpp.obj: ..\extras.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\cmake-build-debug-visual-studio\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/3355_TP4.dir/extras.cpp.obj"
	C:\PROGRA~2\MICROS~1\2019\COMMUN~1\VC\Tools\MSVC\1423~1.281\bin\Hostx86\x86\cl.exe @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) /FoCMakeFiles\3355_TP4.dir\extras.cpp.obj /FdCMakeFiles\3355_TP4.dir\ /FS -c "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\extras.cpp"
<<

CMakeFiles\3355_TP4.dir\extras.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/3355_TP4.dir/extras.cpp.i"
	C:\PROGRA~2\MICROS~1\2019\COMMUN~1\VC\Tools\MSVC\1423~1.281\bin\Hostx86\x86\cl.exe > CMakeFiles\3355_TP4.dir\extras.cpp.i @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\extras.cpp"
<<

CMakeFiles\3355_TP4.dir\extras.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/3355_TP4.dir/extras.cpp.s"
	C:\PROGRA~2\MICROS~1\2019\COMMUN~1\VC\Tools\MSVC\1423~1.281\bin\Hostx86\x86\cl.exe @<<
 /nologo /TP $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) /FoNUL /FAs /FaCMakeFiles\3355_TP4.dir\extras.cpp.s /c "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\extras.cpp"
<<

# Object files for target 3355_TP4
3355_TP4_OBJECTS = \
"CMakeFiles\3355_TP4.dir\main.cpp.obj" \
"CMakeFiles\3355_TP4.dir\object.cpp.obj" \
"CMakeFiles\3355_TP4.dir\parser.cpp.obj" \
"CMakeFiles\3355_TP4.dir\raytracer.cpp.obj" \
"CMakeFiles\3355_TP4.dir\extras.cpp.obj"

# External object files for target 3355_TP4
3355_TP4_EXTERNAL_OBJECTS =

3355_TP4.exe: CMakeFiles\3355_TP4.dir\main.cpp.obj
3355_TP4.exe: CMakeFiles\3355_TP4.dir\object.cpp.obj
3355_TP4.exe: CMakeFiles\3355_TP4.dir\parser.cpp.obj
3355_TP4.exe: CMakeFiles\3355_TP4.dir\raytracer.cpp.obj
3355_TP4.exe: CMakeFiles\3355_TP4.dir\extras.cpp.obj
3355_TP4.exe: CMakeFiles\3355_TP4.dir\build.make
3355_TP4.exe: CMakeFiles\3355_TP4.dir\objects1.rsp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\cmake-build-debug-visual-studio\CMakeFiles" --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable 3355_TP4.exe"
	"C:\Program Files\JetBrains\CLion 2019.3.5\bin\cmake\win\bin\cmake.exe" -E vs_link_exe --intdir=CMakeFiles\3355_TP4.dir --rc=C:\PROGRA~2\WI3CF2~1\10\bin\100183~1.0\x86\rc.exe --mt=C:\PROGRA~2\WI3CF2~1\10\bin\100183~1.0\x86\mt.exe --manifests  -- C:\PROGRA~2\MICROS~1\2019\COMMUN~1\VC\Tools\MSVC\1423~1.281\bin\Hostx86\x86\link.exe /nologo @CMakeFiles\3355_TP4.dir\objects1.rsp @<<
 /out:3355_TP4.exe /implib:3355_TP4.lib /pdb:"C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\cmake-build-debug-visual-studio\3355_TP4.pdb" /version:0.0  /machine:X86 /debug /INCREMENTAL /subsystem:console kernel32.lib user32.lib gdi32.lib winspool.lib shell32.lib ole32.lib oleaut32.lib uuid.lib comdlg32.lib advapi32.lib 
<<

# Rule to build all files generated by this target.
CMakeFiles\3355_TP4.dir\build: 3355_TP4.exe

.PHONY : CMakeFiles\3355_TP4.dir\build

CMakeFiles\3355_TP4.dir\clean:
	$(CMAKE_COMMAND) -P CMakeFiles\3355_TP4.dir\cmake_clean.cmake
.PHONY : CMakeFiles\3355_TP4.dir\clean

CMakeFiles\3355_TP4.dir\depend:
	$(CMAKE_COMMAND) -E cmake_depends "NMake Makefiles" "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master" "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master" "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\cmake-build-debug-visual-studio" "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\cmake-build-debug-visual-studio" "C:\Users\Mo\Documents\7 - Work\projects remem\global-illumination-master\global-illumination-master\cmake-build-debug-visual-studio\CMakeFiles\3355_TP4.dir\DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles\3355_TP4.dir\depend
