#pragma once

#include <QtWidgets/QWidget>

#include "PortType.hpp"
#include "NodeData.hpp"
#include "Serializable.hpp"
#include "NodeGeometry.hpp"
#include "NodeStyle.hpp"
#include "NodePainterDelegate.hpp"
#include "Export.hpp"
#include "memory.hpp"

namespace QtNodes
{

enum class NodeValidationState
{
  Valid,
  Warning,
  Error
};

class StyleCollection;

class NODE_EDITOR_PUBLIC NodeDataModel
  : public QObject
  , public Serializable
{
  Q_OBJECT

public:

  NodeDataModel();

  virtual
  ~NodeDataModel() = default;

  /// Name makes this model unique
  virtual QString
  name() const = 0;

public:

  QJsonObject
  save() const override;

public:

  virtual
  unsigned int nPorts(PortType portType) const = 0;

  virtual
  NodeDataType dataType(PortType portType, PortIndex portIndex) const = 0;

public:

  enum class ConnectionPolicy
  {
    One,
    Many,
  };

  virtual
  ConnectionPolicy
  portOutConnectionPolicy(PortIndex) const
  {
    return ConnectionPolicy::Many;
  }

  NodeStyle const&
  nodeStyle() const;

  void
  setNodeStyle(NodeStyle const& style);

public:

  /// Triggers the algorithm
  virtual
  void
  setInData(std::shared_ptr<NodeData> nodeData,
            PortIndex port) = 0;

  virtual
  std::shared_ptr<NodeData>
  outData(PortIndex port) = 0;

  virtual
  QWidget *
  embeddedWidget() = 0;

  virtual
  bool
  resizable() const { return false; }

  virtual
  NodeValidationState
  validationState() const { return NodeValidationState::Valid; }

  virtual
  QString
  validationMessage() const { return QString(""); }

  virtual
  NodePainterDelegate* painterDelegate() const { return nullptr; }

signals:

  void
  dataUpdated(PortIndex index);

  void
  dataInvalidated(PortIndex index);

  void
  computingStarted();

  void
  computingFinished();

  void embeddedWidgetSizeUpdated();

private:

  NodeStyle _nodeStyle;
};
}
