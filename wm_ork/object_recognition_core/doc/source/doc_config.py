version=''
commithash=''
gittag_short=''
gittag_long=''
git_lastmod=''
github_url=''

breathe_default_project = 'object_recognition_core'
breathe_projects = dict(object_recognition_core='/home/samuel/sara_ws/build/sara_commun/wm_ork/object_recognition_core/doc/../api/xml')

# for release
ork_module_url_root = 'http://wg-perception.github.com/'

intersphinx_mapping = {'orkcapture': (ork_module_url_root + 'capture', None),
                       'orklinemod': (ork_module_url_root + 'linemod', None),
                       'orkreconstruction': (ork_module_url_root + 'reconstruction', None),
                       'orkros': (ork_module_url_root + 'object_recognition_ros', None),
                       'orktabletop': (ork_module_url_root + 'tabletop', None),
                       'orktod': (ork_module_url_root + 'tod', None),
                       'orktransparentobjects': (ork_module_url_root + 'transparent_objects', None),
                       'orktutorials': (ork_module_url_root + 'ork_tutorials', None),
                       }

programoutput_path = ''.split(';')
