#pragma once

#include <QMainWindow>
#include <tool/ConfigBase.h>
class ToolConfig;

class ConfigWindow : public QMainWindow, public ConfigBase {
Q_OBJECT
  public:
    ConfigWindow(QMainWindow* parent);
  public slots:
    void baseLoadConfig(const ToolConfig& config);
    virtual void loadConfig(const ToolConfig& config) = 0;
    virtual void saveConfig(ToolConfig& config) { }
    void controlsChanged(const QString&) final { controlsChanged(); }
    void controlsChanged(double) final { controlsChanged(); }
    void controlsChanged(float) final { controlsChanged(); }
    void controlsChanged(int) final { controlsChanged(); }
    void controlsChanged(short) final { controlsChanged(); }
    void controlsChanged(char) final { controlsChanged(); }
    void controlsChanged(bool) final { controlsChanged(); }
    virtual void controlsChanged() { }

  protected:
    void saveConfig();
    bool loading_ = false;
};

