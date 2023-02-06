import pyinstaller_versionfile

pyinstaller_versionfile.create_versionfile(
    output_file="cc_versionfile.txt",
    version="2.0.0.2",
    # company_name="GE-Hitachi",
    file_description="Cobra Commander Graphical User Interface",
    # internal_name="Simple App",
    legal_copyright="© GE-Hitachi. All rights reserved.",
    # original_filename="SimpleApp.exe",
    product_name="Cobra Commander",
    translations=[0, 1200]
)