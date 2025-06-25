import os
import shutil
import py_compile


current_folder = os.path.dirname(os.path.abspath(__file__))

examples_source_folder = os.path.join(current_folder)
common_source_folder = os.path.join(current_folder, "Common")

destination_examples_folder = os.path.join(current_folder, "../Student")
destination_common_folder = os.path.join(current_folder, "../Student/Common")

# Create the destination folder if it doesn't exist
os.makedirs(destination_common_folder, exist_ok=True)

# Copy all files from common to Student
for item in os.listdir(common_source_folder):
    source_path = os.path.join(common_source_folder, item)
    dest_path = os.path.join(destination_common_folder, item)
    if os.path.isfile(source_path):
        shutil.copy2(source_path, dest_path)


for item in os.listdir(examples_source_folder):
    source_path = os.path.join(examples_source_folder, item)
    dest_path = os.path.join(destination_examples_folder, item)
    if os.path.isfile(source_path):
        shutil.copy2(source_path, dest_path)


for item in os.listdir(destination_common_folder):
    if item.endswith(".py"):
        file_path = os.path.join(destination_common_folder, item)
        py_compile.compile(file_path, cfile=file_path + "c")

# Remove the .py files after compilation
for item in os.listdir(destination_common_folder):
    if item.endswith(".py"):
        file_path = os.path.join(destination_common_folder, item)
        os.remove(file_path)


destination_teacher_folder = os.path.join(current_folder, "../Teacher")
shutil.copytree(examples_source_folder, destination_teacher_folder, dirs_exist_ok=True)