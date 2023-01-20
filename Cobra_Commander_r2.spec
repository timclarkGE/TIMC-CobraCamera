#-*- mode: python ; coding: utf-8 -*-
block_cipher = None


a = Analysis(
    ['Cobra_Commander_r2.py'],
    pathex=[],
    binaries=[["Cobra_Commander_Support_Files\\vhui64.exe","."],["Cobra_Commander_Support_Files\\vhui32.exe","."],[r'C:\Program Files\Phidgets\Phidget22\phidget22.dll','.'], [r'Cobra_Commander_Support_Files\\Bonjour.msi','.'],[r'Cobra_Commander_Support_Files\\Bonjour64.msi','.'], [r'Cobra_Commander_Support_Files\\CDM212364_Setup.exe','.']],
    datas=[],
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=["Cobra_Commander_Support_Files\\virtualhereHook.py"],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)
pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='Cobra Commander',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    uac_admin=True,
    icon='C:\\Users\\isi\\PycharmProjects\\TIMC-CobraCamera\\Cobra_Commander_Support_Files\\cobra_commander_icon.ico',
    version='Cobra_Commander_Support_Files\\cc_versionfile.txt',
)
