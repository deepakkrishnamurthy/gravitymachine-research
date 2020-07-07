# -*- coding: utf-8 -*-
"""
Created on Fri May 11 19:47:20 2018

@author: Francois
"""


def updateStylePatched(self):
    r = '3px'
    if self.dim:
        fg = '#466e82'
        bg = '#466e82' #dark blue
        border = '#466e82' #dark blue
        # border = '#7cf3ac'
    else:
        fg = '#43b1e5' #font
        bg = '#43b1e5' #bleu clair
        border = '#43b1e5' #bleu clair

    if self.orientation == 'vertical':
        self.vStyle = """DockLabel {
            background-color : %s;
            color : %s;
            border-top-right-radius: 0px;
            border-top-left-radius: %s;
            border-bottom-right-radius: 0px;
            border-bottom-left-radius: %s;
            border-width: 0px;
            border-right: 2px solid %s;
            padding-top: 3px;
            padding-bottom: 3px;
            font-size: 12px;
        }""" % (bg, fg, r, r, border)
        self.setStyleSheet(self.vStyle)
    else:
        self.hStyle = """DockLabel {
            background-color : %s;
            color : %s;
            border-top-right-radius: %s;
            border-top-left-radius: %s;
            border-bottom-right-radius: 0px;
            border-bottom-left-radius: 0px;
            border-bottom: 0px solid %s;
            padding-left: 13px;
            padding-right: 13px;           
            font-size: 12px
        }""" % (bg, fg, r, r, border)
        self.setStyleSheet(self.hStyle)
        