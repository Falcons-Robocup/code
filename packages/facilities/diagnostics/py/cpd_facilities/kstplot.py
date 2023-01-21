# Copyright 2021 Jeffrey van Pernis (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python3
import argparse
import subprocess
import tarfile
import tempfile
import textwrap
from datetime import datetime
from pathlib import Path


# ----------------------------------------------------------------
# Global variables
#

this_path = Path(__file__).resolve()

install_path = Path().home() / '.local/share'

kstplot_desktop_path = install_path / 'applications/kstplot.desktop'
kstplot_desktop_content = f"""
[Desktop Entry]
Encoding=UTF-8
Version=1.0
Type=Application
Terminal=false
Exec={this_path} run %u
Name=KST Plot Viewer
Comment=KST Plot File Handler
Icon=kst
Categories=Application;
MimeType=application/x-kstplot;
""".strip()

kstplot_package_path = install_path / 'mime/packages/kstplot.xml'
kstplot_package_content = """
<?xml version="1.0" encoding="UTF-8"?>
<mime-info xmlns="http://www.freedesktop.org/standards/shared-mime-info">
   <mime-type type="application/x-kstplot">
     <comment>KST Plot Viewer</comment>
     <glob pattern="*.kstplot"/>
     <icon name="kst"/>
   </mime-type>
</mime-info>
""".strip()

mimetypes_path = Path.home() / '.config/mimeapps.list'
mimetypes_line = "application/x-kstplot=kstplot.desktop;\n"



# ----------------------------------------------------------------
# Commands
#

def install():
  """Write the MIME config files to recognize what to do with .kstplot files"""

  kstplot_desktop_path.parent.mkdir(parents=True, exist_ok=True)
  kstplot_desktop_path.write_text(kstplot_desktop_content)

  kstplot_package_path.parent.mkdir(parents=True, exist_ok=True)
  kstplot_package_path.write_text(kstplot_package_content)

  mimetypes_path.parent.mkdir(parents=True, exist_ok=True)
  if mimetypes_path.exists():
    mimetypes_content = mimetypes_path.read_text()
  else:
    mimetypes_content = "[Added Associations]\n"

  if mimetypes_line not in mimetypes_content:
    mimetypes_content += mimetypes_line

  mimetypes_path.write_text(mimetypes_content)

  subprocess.run(['update-mime-database', install_path / 'mime'])


def uninstall():
  """Undo any modifications made by this script when writing the MIME config files"""

  kstplot_desktop_path.unlink()
  kstplot_package_path.unlink()

  mimetypes_content = mimetypes_path.read_text()
  mimetypes_content.replace(mimetypes_line, '')
  mimetypes_path.write_text(mimetypes_content)

  subprocess.run(['update-mime-database', install_path / 'mime'])


def run(plotfile):
  """Extract the .kstplot archive file into a temporary directory and open kst2"""

  with tempfile.TemporaryDirectory(prefix='kstplot-') as tmp_dir:
      with tarfile.open(plotfile, "r:gz") as tar:
          tar.extract('plot.kst', tmp_dir)
          tar.extract('plot.txt', tmp_dir)

      kst_file = tmp_dir + '/plot.kst'
      subprocess.run(['kst2', kst_file])


def create(textfile, start=None):
  """Create a .kstplot archive file from the specified textfile and the latest KST project file"""

  if not start:
    start = datetime.now()

  templatefile = this_path.parent.parent / 'cpd/plot.kst'
  plotfile = Path(textfile).with_suffix('.kstplot')

  with tempfile.TemporaryDirectory(prefix='kstplot-') as tmp_dir:
    projectfile = Path(tmp_dir) / 'plot.kst'

    # Replace the start time template in the project file with the one we have received
    project = templatefile.read_text()
    project = project.replace('{PLOT_START_TIME}', start.isoformat() + 'Z')
    projectfile.write_text(project)

    # Generate the kstplot archive file
    with tarfile.open(plotfile, "w:gz") as tar:
      tar.add(projectfile, 'plot.kst', recursive=False)
      tar.add(textfile, 'plot.txt', recursive=False)

  return plotfile



# ----------------------------------------------------------------
# Main
#

if __name__ == '__main__':

  description="This is a utility script which helps to plot the result of a robot move using KST"

  example_text = textwrap.dedent("""
  examples:
    kstplot.py install --help
    kstplot.py install
    kstplot.py create plotRobotVel.txt
    kstplot.py run plotRobotVel.kstplot
  """)

  parser = argparse.ArgumentParser(description=description, epilog=example_text, formatter_class=argparse.RawDescriptionHelpFormatter)
  cmdparsers = parser.add_subparsers(dest='command')

  cmd_install = cmdparsers.add_parser('install', description=install.__doc__)
  cmd_install.set_defaults(func=lambda args: install())

  cmd_uninstall = cmdparsers.add_parser('uninstall', description=uninstall.__doc__)
  cmd_uninstall.set_defaults(func=lambda args: uninstall())

  run_create = cmdparsers.add_parser('run', description=run.__doc__)
  run_create.add_argument('kstplot', help='An .kstplot archive file which is to be opened with KST')
  run_create.set_defaults(func=lambda args: run(args.kstplot))

  cmd_create = cmdparsers.add_parser('create', description=create.__doc__)
  cmd_create.add_argument('plotfile', help='A text file containing the data for the KST plot')
  cmd_create.set_defaults(func=lambda args: create(args.plotfile))

  args = parser.parse_args()

  if args.command:
    args.func(args)
  else:
    parser.print_help()
