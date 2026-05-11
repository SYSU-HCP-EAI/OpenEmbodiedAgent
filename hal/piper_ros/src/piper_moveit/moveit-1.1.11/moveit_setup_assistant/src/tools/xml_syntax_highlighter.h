/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Bielefeld University, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Robert Haschke */

#pragma once

#include <QtGui/QSyntaxHighlighter>
#include <QRegularExpression>
#include <map>

/** XML SyntaxHighlighter allowing nested highlighting of XML tags */
class XmlSyntaxHighlighter : public QSyntaxHighlighter
{
public:
  XmlSyntaxHighlighter(QTextDocument* parent = nullptr);
  void addTag(const QString& tag, const QTextCharFormat& format, const QString& parent = QString());

protected:
  void highlightBlock(const QString& text) override;

private:
  struct Rule
  {
    QRegularExpression start;
    QRegularExpression end;
    QTextCharFormat format;
    std::map<int, Rule>::const_iterator parent;
  };
  using Rules = std::map<int, Rule>;
  Rules rules;

  Rules::const_iterator highlight(Rules::const_iterator active, QStringRef text, int start, bool search_end, int& end);
};
