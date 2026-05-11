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

#include "xml_syntax_highlighter.h"
#include <assert.h>

XmlSyntaxHighlighter::XmlSyntaxHighlighter(QTextDocument* parent) : QSyntaxHighlighter(parent)
{
}

void XmlSyntaxHighlighter::addTag(const QString& tag, const QTextCharFormat& format, const QString& parent)
{
  const QString start_pattern("<%1.*?/?>");
  Rule rule;
  rule.start = QRegularExpression(start_pattern.arg(tag));
  rule.end = QRegularExpression(QString("</%1>|<%1[^>]*?/>").arg(tag));
  rule.format = format;
  if (!parent.isEmpty())
  {
    QString parent_start = start_pattern.arg(parent);
    rule.parent = std::find_if(rules.begin(), rules.end(), [&](const std::pair<int, Rule>& rule) {
      return rule.second.start.pattern() == parent_start;
    });
  }
  else
    rule.parent = rules.end();

  rules.insert(std::make_pair(rules.size(), rule));
}

XmlSyntaxHighlighter::Rules::const_iterator
XmlSyntaxHighlighter::highlight(Rules::const_iterator active, QStringRef text, int start, bool search_end, int& end)
{
  int offset = end;    // when passed, end indicates the end of the opening expression
  auto next = active;  // return value: active rule at end of text

  if (search_end)  // find end of active rule
  {
    auto match = active->second.end.match(text);
    // when returned, end indicates the end of the closing expression
    end = match.hasMatch() ? match.capturedEnd() : text.size();
    setFormat(start, end, active->second.format);
    if (match.hasMatch())
    {
      text = text.left(match.capturedStart());  // drop text after (and including) closing expression
      next = active->second.parent;
    }
  }
  text = text.mid(offset);  // skip opening expression
  start += offset;          // adjust start by skipped offset
  if (text.isEmpty())
    return next;  // early return

  // highlight remaining text using active's children's rules
  for (auto it = rules.begin(); it != rules.end(); ++it)
  {
    const auto& rule = it->second;
    if (rule.parent != active)
      continue;  // skip wrong rules

    offset = 0;   // (re)start at beginning of (clipped) text
    while (true)  // process all matches of rule
    {
      auto match = rule.start.match(text, offset);
      if (!match.hasMatch())
        break;

      offset = match.capturedEnd() - match.capturedStart();  // mark end of opening expression in passed text
      auto result = highlight(it, text.mid(match.capturedStart()), start + match.capturedStart(), true, offset);
      // returned offset is w.r.t. beginning of _passed_ text: add passed start offset to yield offset w.r.t. text
      offset += match.capturedStart();
      if (result == it)  // text is ending with this rule
      {
        assert(next == active || next == active->second.parent);
        assert(offset == text.size());  // end should mark the end of the text
        next = result;                  // remember return value: active rule at end of text
        break;
      }
    }
  }

  return next;
}

void XmlSyntaxHighlighter::highlightBlock(const QString& text)
{
  Rules::const_iterator active = previousBlockState() < 0 ? rules.end() : rules.find(previousBlockState());
  int unused = 0;
  active = highlight(active, QStringRef(&text, 0, text.size()), 0, active != rules.cend(), unused);
  setCurrentBlockState(active != rules.cend() ? active->first : -1);
}
