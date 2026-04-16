/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "widgets/ratio_layouted_frame.h"

#include <assert.h>
#include <QMouseEvent>
#include <QPainter>



RatioLayoutedFrame::RatioLayoutedFrame(QWidget* parent, Qt::WindowFlags flags)
    : QFrame(parent, flags), outer_layout_(NULL), aspect_ratio_(4, 3), smoothImage_(false) {
  connect(this, SIGNAL(delayed_update()), this, SLOT(update()), Qt::QueuedConnection);
}

RatioLayoutedFrame::~RatioLayoutedFrame() {
}

const QImage& RatioLayoutedFrame::getImage() const {
  return qimage_;
}

QImage RatioLayoutedFrame::getImageCopy() const {
  QImage img;
  qimage_mutex_.lock();
  img = qimage_.copy();
  qimage_mutex_.unlock();
  return img;
}

void RatioLayoutedFrame::setImage(const QImage& image)  //, QMutex* image_mutex)
{
  qimage_mutex_.lock();
  qimage_ = image.copy();
  setAspectRatio(qimage_.width(), qimage_.height());
  // 核心优化：收到新图时立即预计算缩放图，避免在 paintEvent 中重复计算
  if (smoothImage_ && !qimage_.isNull() && !contentsRect().isEmpty()) {
      if (contentsRect().width() != qimage_.width() || contentsRect().height() != qimage_.height()) {
          scaled_image_ = qimage_.scaled(contentsRect().width(), contentsRect().height(),
                                        Qt::KeepAspectRatio, Qt::SmoothTransformation);
      } else {
          scaled_image_ = QImage(); // 尺寸一致不需要缩放
      }
  } else {
      scaled_image_ = QImage();
  }
  qimage_mutex_.unlock();
  emit delayed_update();
}

void RatioLayoutedFrame::resizeToFitAspectRatio() {
  if (in_resize_) return;
  in_resize_ = true;
  
  QRect rect = contentsRect();
  if (rect.isEmpty()) {
    in_resize_ = false;
    return;
  }

  // reduce longer edge to aspect ration
  double target_w;
  double target_h;

  if (outer_layout_) {
    target_w = outer_layout_->contentsRect().width();
    target_h = outer_layout_->contentsRect().height();
  } else {
    target_w = rect.width();
    target_h = rect.height();
  }

  if (target_w <= 0 || target_h <= 0) {
    in_resize_ = false;
    return;
  }

  double layout_ar = target_w / target_h;
  const double image_ar = double(aspect_ratio_.width()) / double(aspect_ratio_.height());
  if (image_ar <= 0) {
    in_resize_ = false;
    return;
  }

  if (layout_ar > image_ar) {
    target_w = target_h * image_ar;
  } else {
    target_h = target_w / image_ar;
  }
  
  int final_w = int(target_w + 0.5);
  int final_h = int(target_h + 0.5);

  // resize taking the border line into account
  int border = lineWidth();
  int current_w = this->width();
  int current_h = this->height();
  
  if (abs(current_w - (final_w + 2 * border)) > 1 || abs(current_h - (final_h + 2 * border)) > 1) {
    resize(final_w + 2 * border, final_h + 2 * border);
  }
  
  in_resize_ = false;
}

void RatioLayoutedFrame::setOuterLayout(QHBoxLayout* outer_layout) {
  outer_layout_ = outer_layout;
}

void RatioLayoutedFrame::setInnerFrameMinimumSize(const QSize& size) {
  int border = lineWidth();
  QSize new_size = size;
  new_size += QSize(2 * border, 2 * border);
  setMinimumSize(new_size);
  emit delayed_update();
}

void RatioLayoutedFrame::setInnerFrameMaximumSize(const QSize& size) {
  int border = lineWidth();
  QSize new_size = size;
  new_size += QSize(2 * border, 2 * border);
  setMaximumSize(new_size);
  emit delayed_update();
}

void RatioLayoutedFrame::setInnerFrameFixedSize(const QSize& size) {
  setInnerFrameMinimumSize(size);
  setInnerFrameMaximumSize(size);
}

void RatioLayoutedFrame::setAspectRatio(unsigned short width, unsigned short height) {
  int divisor = greatestCommonDivisor(width, height);
  if (divisor != 0) {
    aspect_ratio_.setWidth(width / divisor);
    aspect_ratio_.setHeight(height / divisor);
  }
}

void RatioLayoutedFrame::paintEvent(QPaintEvent* event) {
  QPainter painter(this);
  qimage_mutex_.lock();
  if (!qimage_.isNull() && !contentsRect().isEmpty()) {
    if (!smoothImage_) {
      painter.drawImage(contentsRect(), qimage_);
    } else {
      // 性能优化：优先使用缓存的缩放图
      if (!scaled_image_.isNull()) {
          painter.drawImage(contentsRect(), scaled_image_);
      } else {
          painter.drawImage(contentsRect(), qimage_);
      }
    }
  } else {
    // default image with gradient
    QLinearGradient gradient(0, 0, frameRect().width(), frameRect().height());
    gradient.setColorAt(0, Qt::white);
    gradient.setColorAt(1, Qt::black);
    painter.setBrush(gradient);
    painter.drawRect(0, 0, frameRect().width() + 1, frameRect().height() + 1);
  }
  qimage_mutex_.unlock();
}

void RatioLayoutedFrame::resizeEvent(QResizeEvent* event) {
  QFrame::resizeEvent(event);
  resizeToFitAspectRatio();
  
  // 核心优化：尺寸变化时更新缓存图
  qimage_mutex_.lock();
  if (smoothImage_ && !qimage_.isNull() && !contentsRect().isEmpty()) {
      if (contentsRect().width() != qimage_.width() || contentsRect().height() != qimage_.height()) {
          scaled_image_ = qimage_.scaled(contentsRect().width(), contentsRect().height(),
                                        Qt::KeepAspectRatio, Qt::SmoothTransformation);
      } else {
          scaled_image_ = QImage();
      }
  } else {
      scaled_image_ = QImage();
  }
  qimage_mutex_.unlock();
}

int RatioLayoutedFrame::greatestCommonDivisor(int a, int b) {
  if (b == 0) {
    return a;
  }
  return greatestCommonDivisor(b, a % b);
}

void RatioLayoutedFrame::mousePressEvent(QMouseEvent* mouseEvent) {
  if (mouseEvent->button() == Qt::LeftButton) {
    emit mouseLeft(mouseEvent->x(), mouseEvent->y());
  }
  QFrame::mousePressEvent(mouseEvent);
}

void RatioLayoutedFrame::onSmoothImageChanged(bool checked) {
  smoothImage_ = checked;
  // 状态变化时触发重绘和缓存刷新
  update();
}
