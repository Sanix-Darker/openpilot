#pragma once

#include <deque>
#include <QCheckBox>
#include <QComboBox>
#include <QHeaderView>
#include <QLineEdit>
#include <QTableView>

#include "tools/cabana/dbcmanager.h"
#include "tools/cabana/streams/abstractstream.h"

class HeaderView : public QHeaderView {
public:
  HeaderView(Qt::Orientation orientation, QWidget *parent = nullptr) : QHeaderView(orientation, parent) {}
  QSize sectionSizeFromContents(int logicalIndex) const override;
  void paintSection(QPainter *painter, const QRect &rect, int logicalIndex) const;
};

class HistoryLogModel : public QAbstractTableModel {
  Q_OBJECT

public:
  HistoryLogModel(QObject *parent) : QAbstractTableModel(parent) {}
  void setMessage(const QString &message_id);
  void updateState();
  void setFilter(int sig_idx, const QString &value, std::function<bool(double, double)> cmp);
  QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
  QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
  void fetchMore(const QModelIndex &parent) override;
  inline bool canFetchMore(const QModelIndex &parent) const override { return has_more_data; }
  int rowCount(const QModelIndex &parent = QModelIndex()) const override { return messages.size(); }
  int columnCount(const QModelIndex &parent = QModelIndex()) const override {
    return display_signals_mode && !sigs.empty() ? sigs.size() + 1 : 2;
  }
  void updateColors();
  void refresh();

public slots:
  void setDisplayType(int type);
  void setDynamicMode(int state);
  void segmentsMerged();

public:
  struct Message {
    uint64_t mono_time = 0;
    QVector<double> sig_values;
    QByteArray data;
    QVector<QColor> colors;
  };

  template <class InputIt>
  std::deque<HistoryLogModel::Message> fetchData(InputIt first, InputIt last, uint64_t min_time);
  std::deque<Message> fetchData(uint64_t from_time, uint64_t min_time = 0);

  QString msg_id;
  HexColors hex_colors;
  bool has_more_data = true;
  const int batch_size = 50;
  int filter_sig_idx = -1;
  double filter_value = 0;
  uint64_t last_fetch_time = 0;
  std::function<bool(double, double)> filter_cmp = nullptr;
  std::deque<Message> messages;
  std::vector<const Signal*> sigs;
  bool dynamic_mode = true;
  bool display_signals_mode = true;
};

class LogsWidget : public QWidget {
  Q_OBJECT

public:
  LogsWidget(QWidget *parent);
  void setMessage(const QString &message_id);
  void updateState() {if (dynamic_mode->isChecked()) model->updateState(); }
  void showEvent(QShowEvent *event) override { if (dynamic_mode->isChecked()) model->refresh(); }

signals:
  void openChart(const QString &msg_id, const Signal *sig);

private slots:
  void setFilter();

private:
  QTableView *logs;
  HistoryLogModel *model;
  QCheckBox *dynamic_mode;
  QComboBox *signals_cb, *comp_box, *display_type_cb;
  QLineEdit *value_edit;
  QWidget *filters_widget;
};
