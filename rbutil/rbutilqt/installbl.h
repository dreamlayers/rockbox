/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 *   Copyright (C) 2007 by Dominik Wenger
 *   $Id: installbl.h 14027 2007-07-27 17:42:49Z domonoky $
 *
 * All files in this archive are subject to the GNU General Public License.
 * See the file COPYING in the source tree root for full license agreement.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

#ifndef INSTALLBL_H
#define INSTALLBL_H

#include <QtGui>

#include <QSettings>

#include "ui_installbootloaderfrm.h"
#include "progressloggergui.h"

#include "installbootloader.h"
#include "irivertools/irivertools.h"

class InstallBl : public QDialog
{
    Q_OBJECT
    public:
        InstallBl(QWidget *parent = 0);
        void setProxy(QUrl);
        void setMountPoint(QString);
        void setOFPath(QString);
        void setUserSettings(QSettings*);
        void setDeviceSettings(QSettings*);

    public slots:
        void accept(void);

    private:
        Ui::InstallBootloaderFrm ui;
        ProgressLoggerGui* logger;
        QUrl proxy;
        QSettings *devices;
        QSettings *userSettings;
        QHttp *download;
        QFile *target;
        QString file;
        QString fileName;
        QString mountPoint;
        QString m_OrigFirmware;
        BootloaderInstaller* binstaller;
        bool needextrafile;

    private slots:
        void browseFolder(void);
        void browseOF(void);
        void done(bool);

};


#endif
