#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Mark Moll, Ioan Sucan, Luis G. Torres

from sys import argv, exit
from os.path import basename, splitext
import sqlite3
import datetime
import matplotlib

matplotlib.use("pdf")
from matplotlib import __version__ as matplotlibversion
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.pyplot as plt
import numpy as np
from math import floor
from optparse import OptionParser, OptionGroup

# Given a text line, split it into tokens (by space) and return the token
# at the desired index. Additionally, test that some expected tokens exist.
# Return None if they do not.
def readLogValue(filevar, desired_token_index, expected_tokens):
    start_pos = filevar.tell()
    tokens = filevar.readline().split()
    for token_index in expected_tokens:
        if not tokens[token_index] == expected_tokens[token_index]:
            # undo the read, if we failed to parse.
            filevar.seek(start_pos)
            return None
    return tokens[desired_token_index]


def readOptionalLogValue(filevar, desired_token_index, expected_tokens={}):
    return readLogValue(filevar, desired_token_index, expected_tokens)


def readRequiredLogValue(name, filevar, desired_token_index, expected_tokens={}):
    result = readLogValue(filevar, desired_token_index, expected_tokens)
    if result == None:
        raise Exception("Unable to read " + name)
    return result


def ensurePrefix(line, prefix):
    if not line.startswith(prefix):
        raise Exception("Expected prefix " + prefix + " was not found")
    return line


def readOptionalMultilineValue(filevar):
    start_pos = filevar.tell()
    line = filevar.readline()
    if not line.startswith("<<<|"):
        filevar.seek(start_pos)
        return None
    value = ""
    line = filevar.readline()
    while not line.startswith("|>>>"):
        value = value + line
        line = filevar.readline()
        if line == None:
            raise Exception("Expected token |>>> missing")
    return value


def readRequiredMultilineValue(filevar):
    ensurePrefix(filevar.readline(), "<<<|")
    value = ""
    line = filevar.readline()
    while not line.startswith("|>>>"):
        value = value + line
        line = filevar.readline()
        if line == None:
            raise Exception("Expected token |>>> missing")
    return value


def readBenchmarkLog(dbname, filenames):
    """Parse benchmark log files and store the parsed data in a sqlite3 database."""

    def isInvalidValue(value):
        return len(value) == 0 or value in ["nan", "-nan", "inf", "-inf"]

    conn = sqlite3.connect(dbname)
    c = conn.cursor()
    c.execute("PRAGMA FOREIGN_KEYS = ON")

    # create all tables if they don't already exist
    c.executescript(
        """CREATE TABLE IF NOT EXISTS experiments
        (id INTEGER PRIMARY KEY ON CONFLICT REPLACE AUTOINCREMENT, name VARCHAR(512),
        totaltime REAL, timelimit REAL, memorylimit REAL, runcount INTEGER,
        version VARCHAR(128), hostname VARCHAR(1024), cpuinfo TEXT,
        date DATETIME, seed INTEGER, setup TEXT);
        CREATE TABLE IF NOT EXISTS plannerConfigs
        (id INTEGER PRIMARY KEY AUTOINCREMENT,
        name VARCHAR(512) NOT NULL, settings TEXT);
        CREATE TABLE IF NOT EXISTS enums
        (name VARCHAR(512), value INTEGER, description TEXT,
        PRIMARY KEY (name, value));
        CREATE TABLE IF NOT EXISTS runs
        (id INTEGER PRIMARY KEY AUTOINCREMENT, experimentid INTEGER, plannerid INTEGER,
        FOREIGN KEY (experimentid) REFERENCES experiments(id) ON DELETE CASCADE,
        FOREIGN KEY (plannerid) REFERENCES plannerConfigs(id) ON DELETE CASCADE);
        CREATE TABLE IF NOT EXISTS progress
        (runid INTEGER, time REAL, PRIMARY KEY (runid, time),
        FOREIGN KEY (runid) REFERENCES runs(id) ON DELETE CASCADE)"""
    )

    # add placeholder entry for all_experiments
    allExperimentsName = "all_experiments"
    allExperimentsValues = {
        "totaltime": 0.0,
        "timelimit": 0.0,
        "memorylimit": 0.0,
        "runcount": 0,
        "version": "0.0.0",
        "hostname": "",
        "cpuinfo": "",
        "date": 0,
        "seed": 0,
        "setup": "",
    }
    addAllExperiments = len(filenames) > 0
    if addAllExperiments:
        c.execute(
            "INSERT INTO experiments VALUES (?,?,?,?,?,?,?,?,?,?,?,?)",
            (None, allExperimentsName) + tuple(allExperimentsValues.values()),
        )
        allExperimentsId = c.lastrowid

    for i, filename in enumerate(filenames):
        print("Processing " + filename)
        logfile = open(filename, "r")
        start_pos = logfile.tell()
        libname = readOptionalLogValue(logfile, 0, {1: "version"})
        if libname == None:
            libname = "OMPL"
        logfile.seek(start_pos)
        version = readOptionalLogValue(logfile, -1, {1: "version"})
        if version == None:
            # set the version number to make Planner Arena happy
            version = "0.0.0"
        version = " ".join([libname, version])
        expname = readRequiredLogValue(
            "experiment name", logfile, -1, {0: "Experiment"}
        )
        hostname = readRequiredLogValue("hostname", logfile, -1, {0: "Running"})
        date = " ".join(ensurePrefix(logfile.readline(), "Starting").split()[2:])
        expsetup = readRequiredMultilineValue(logfile)
        cpuinfo = readOptionalMultilineValue(logfile)
        rseed = int(
            readRequiredLogValue("random seed", logfile, 0, {-2: "random", -1: "seed"})
        )
        timelimit = float(
            readRequiredLogValue(
                "time limit", logfile, 0, {-3: "seconds", -2: "per", -1: "run"}
            )
        )
        memorylimit = float(
            readRequiredLogValue(
                "memory limit", logfile, 0, {-3: "MB", -2: "per", -1: "run"}
            )
        )
        nrrunsOrNone = readOptionalLogValue(
            logfile, 0, {-3: "runs", -2: "per", -1: "planner"}
        )
        nrruns = -1
        if nrrunsOrNone != None:
            nrruns = int(nrrunsOrNone)
            allExperimentsValues["runcount"] += nrruns
        totaltime = float(
            readRequiredLogValue(
                "total time", logfile, 0, {-3: "collect", -2: "the", -1: "data"}
            )
        )
        # fill in fields of all_experiments
        allExperimentsValues["totaltime"] += totaltime
        allExperimentsValues["memorylimit"] = max(
            allExperimentsValues["memorylimit"], totaltime
        )
        allExperimentsValues["timelimit"] = max(
            allExperimentsValues["timelimit"], totaltime
        )
        # copy the fields of the first file to all_experiments so that they are not empty
        if i == 0:
            allExperimentsValues["version"] = version
            allExperimentsValues["date"] = date
            allExperimentsValues["setup"] = expsetup
            allExperimentsValues["hostname"] = hostname
            allExperimentsValues["cpuinfo"] = cpuinfo
        numEnums = 0
        numEnumsOrNone = readOptionalLogValue(logfile, 0, {-2: "enum"})
        if numEnumsOrNone != None:
            numEnums = int(numEnumsOrNone)
        for i in range(numEnums):
            enum = logfile.readline()[:-1].split("|")
            c.execute('SELECT * FROM enums WHERE name IS "%s"' % enum[0])
            if c.fetchone() == None:
                for j in range(len(enum) - 1):
                    c.execute(
                        "INSERT INTO enums VALUES (?,?,?)", (enum[0], j, enum[j + 1])
                    )
        c.execute(
            "INSERT INTO experiments VALUES (?,?,?,?,?,?,?,?,?,?,?,?)",
            (
                None,
                expname,
                totaltime,
                timelimit,
                memorylimit,
                nrruns,
                version,
                hostname,
                cpuinfo,
                date,
                rseed,
                expsetup,
            ),
        )
        experimentId = c.lastrowid
        numPlanners = int(
            readRequiredLogValue("planner count", logfile, 0, {-1: "planners"})
        )
        for i in range(numPlanners):
            plannerName = logfile.readline()[:-1]
            print("Parsing data for " + plannerName)

            # read common data for planner
            numCommon = int(logfile.readline().split()[0])
            settings = ""
            for j in range(numCommon):
                settings = settings + logfile.readline() + ";"

            # find planner id
            c.execute(
                "SELECT id FROM plannerConfigs WHERE (name=? AND settings=?)",
                (
                    plannerName,
                    settings,
                ),
            )
            p = c.fetchone()
            if p == None:
                c.execute(
                    "INSERT INTO plannerConfigs VALUES (?,?,?)",
                    (
                        None,
                        plannerName,
                        settings,
                    ),
                )
                plannerId = c.lastrowid
            else:
                plannerId = p[0]

            # get current column names
            c.execute("PRAGMA table_info(runs)")
            columnNames = [col[1] for col in c.fetchall()]

            # read properties and add columns as necessary
            numProperties = int(logfile.readline().split()[0])
            propertyNames = ["experimentid", "plannerid"]
            for j in range(numProperties):
                field = logfile.readline().split()
                propertyType = field[-1]
                propertyName = "_".join(field[:-1])
                if propertyName not in columnNames:
                    c.execute(
                        "ALTER TABLE runs ADD %s %s" % (propertyName, propertyType)
                    )
                propertyNames.append(propertyName)
            # read measurements
            insertFmtStr = (
                "INSERT INTO runs ("
                + ",".join(propertyNames)
                + ") VALUES ("
                + ",".join("?" * len(propertyNames))
                + ")"
            )
            numRuns = int(logfile.readline().split()[0])
            runIds = []
            for j in range(numRuns):
                runValues = [
                    None if isInvalidValue(x) else x
                    for x in logfile.readline().split("; ")[:-1]
                ]
                values = tuple([experimentId, plannerId] + runValues)
                c.execute(insertFmtStr, values)
                # extract primary key of each run row so we can reference them
                # in the planner progress data table if needed
                runIds.append(c.lastrowid)
                # add all run data to all_experiments
                if addAllExperiments:
                    values = tuple([allExperimentsId, plannerId] + runValues)
                    c.execute(insertFmtStr, values)

            nextLine = logfile.readline().strip()

            # read planner progress data if it's supplied
            if nextLine != ".":
                # get current column names
                c.execute("PRAGMA table_info(progress)")
                columnNames = [col[1] for col in c.fetchall()]

                # read progress properties and add columns as necesary
                numProgressProperties = int(nextLine.split()[0])
                progressPropertyNames = ["runid"]
                for i in range(numProgressProperties):
                    field = logfile.readline().split()
                    progressPropertyType = field[-1]
                    progressPropertyName = "_".join(field[:-1])
                    if progressPropertyName not in columnNames:
                        c.execute(
                            "ALTER TABLE progress ADD %s %s"
                            % (progressPropertyName, progressPropertyType)
                        )
                    progressPropertyNames.append(progressPropertyName)
                # read progress measurements
                insertFmtStr = (
                    "INSERT INTO progress ("
                    + ",".join(progressPropertyNames)
                    + ") VALUES ("
                    + ",".join("?" * len(progressPropertyNames))
                    + ")"
                )
                numRuns = int(logfile.readline().split()[0])
                for j in range(numRuns):
                    dataSeries = logfile.readline().split(";")[:-1]
                    for dataSample in dataSeries:
                        values = tuple(
                            [runIds[j]]
                            + [
                                None if isInvalidValue(x) else x
                                for x in dataSample.split(",")[:-1]
                            ]
                        )
                        try:
                            c.execute(insertFmtStr, values)
                        except sqlite3.IntegrityError:
                            print(
                                "Ignoring duplicate progress data. Consider increasing ompl::tools::Benchmark::Request::timeBetweenUpdates."
                            )
                            pass

                logfile.readline()
        logfile.close()

    if addAllExperiments:
        updateString = "UPDATE experiments SET"
        for i, (key, val) in enumerate(allExperimentsValues.items()):
            if i > 0:
                updateString += ","
            updateString += " " + str(key) + "='" + str(val) + "'"
        updateString += "WHERE id='" + str(allExperimentsId) + "'"
        c.execute(updateString)
    conn.commit()
    c.close()


def plotAttribute(cur, planners, attribute, typename):
    """Create a plot for a particular attribute. It will include data for
    all planners that have data for this attribute."""
    labels = []
    measurements = []
    nanCounts = []
    if typename == "ENUM":
        cur.execute('SELECT description FROM enums where name IS "%s"' % attribute)
        descriptions = [t[0] for t in cur.fetchall()]
        numValues = len(descriptions)
    for planner in planners:
        cur.execute(
            "SELECT %s FROM runs WHERE plannerid = %s AND %s IS NOT NULL"
            % (attribute, planner[0], attribute)
        )
        measurement = [t[0] for t in cur.fetchall() if t[0] != None]
        if len(measurement) > 0:
            cur.execute(
                "SELECT count(*) FROM runs WHERE plannerid = %s AND %s IS NULL"
                % (planner[0], attribute)
            )
            nanCounts.append(cur.fetchone()[0])
            labels.append(planner[1])
            if typename == "ENUM":
                scale = 100.0 / len(measurement)
                measurements.append(
                    [measurement.count(i) * scale for i in range(numValues)]
                )
            else:
                measurements.append(measurement)

    if len(measurements) == 0:
        print('Skipping "%s": no available measurements' % attribute)
        return

    plt.clf()
    ax = plt.gca()
    if typename == "ENUM":
        width = 0.5
        measurements = np.transpose(np.vstack(measurements))
        colsum = np.sum(measurements, axis=1)
        rows = np.where(colsum != 0)[0]
        heights = np.zeros((1, measurements.shape[1]))
        ind = range(measurements.shape[1])
        legend_labels = []
        for i in rows:
            plt.bar(
                ind,
                measurements[i],
                width,
                bottom=heights[0],
                color=matplotlib.cm.hot(int(floor(i * 256 / numValues))),
                label=descriptions[i],
            )
            heights = heights + measurements[i]
        xtickNames = plt.xticks(
            [x + width / 2.0 for x in ind], labels, rotation=30, fontsize=8, ha="right"
        )
        ax.set_ylabel(attribute.replace("_", " ") + " (%)")
        box = ax.get_position()
        ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])
        props = matplotlib.font_manager.FontProperties()
        props.set_size("small")
        ax.legend(loc="center left", bbox_to_anchor=(1, 0.5), prop=props)
    elif typename == "BOOLEAN":
        width = 0.5
        measurementsPercentage = [sum(m) * 100.0 / len(m) for m in measurements]
        ind = range(len(measurements))
        plt.bar(ind, measurementsPercentage, width)
        ### uncommenting this line will remove the term 'kConfigDefault' from the labels for OMPL Solvers.
        ### Fits situations where you need more control in the plot, such as in an academic publication for example
        # labels = [l.replace('kConfigDefault', '') for l in labels]

        xtickNames = plt.xticks(
            [x + width / 2.0 for x in ind], labels, rotation=30, fontsize=8, ha="right"
        )
        ax.set_ylabel(attribute.replace("_", " ") + " (%)")
        plt.subplots_adjust(
            bottom=0.3
        )  # Squish the plot into the upper 2/3 of the page.  Leave room for labels
    else:
        if int(matplotlibversion.split(".")[0]) < 1:
            plt.boxplot(measurements, notch=0, sym="k+", vert=1, whis=1.5)
        else:
            plt.boxplot(
                measurements, notch=0, sym="k+", vert=1, whis=1.5, bootstrap=1000
            )
        ax.set_ylabel(attribute.replace("_", " "))

        # xtickNames = plt.xticks(labels, rotation=30, fontsize=10)
        # plt.subplots_adjust(bottom=0.3) # Squish the plot into the upper 2/3 of the page.  Leave room for labels

        ### uncommenting this line will remove the term 'kConfigDefault' from the labels for OMPL Solvers.
        ### Fits situations where you need more control in the plot, such as in an academic publication for example
        # labels = [l.replace('kConfigDefault', '') for l in labels]

        xtickNames = plt.setp(ax, xticklabels=labels)
        plt.setp(xtickNames, rotation=30, fontsize=8, ha="right")
        for (
            tick
        ) in ax.xaxis.get_major_ticks():  # shrink the font size of the x tick labels
            tick.label.set_fontsize(8)
        plt.subplots_adjust(
            bottom=0.3
        )  # Squish the plot into the upper 2/3 of the page.  Leave room for labels
    ax.set_xlabel("Motion planning algorithm", fontsize=12)
    ax.yaxis.grid(True, linestyle="-", which="major", color="lightgrey", alpha=0.5)
    if max(nanCounts) > 0:
        maxy = max([max(y) for y in measurements])
        for i in range(len(labels)):
            x = i + width / 2 if typename == "BOOLEAN" else i + 1
        ### uncommenting the next line, the number of failed planning attempts will be added to each bar
        # ax.text(x, .95*maxy, str(nanCounts[i]), horizontalalignment='center', size='small')
    plt.show()


def plotProgressAttribute(cur, planners, attribute):
    """Plot data for a single planner progress attribute. Will create an
    average time-plot with error bars of the attribute over all runs for
    each planner."""

    import numpy.ma as ma

    plt.clf()
    ax = plt.gca()
    ax.set_xlabel("time (s)")
    ax.set_ylabel(attribute.replace("_", " "))
    plannerNames = []
    for planner in planners:
        cur.execute(
            """SELECT count(progress.%s) FROM progress INNER JOIN runs
            ON progress.runid = runs.id AND runs.plannerid=%s
            AND progress.%s IS NOT NULL"""
            % (attribute, planner[0], attribute)
        )
        if cur.fetchone()[0] > 0:
            plannerNames.append(planner[1])
            cur.execute(
                """SELECT DISTINCT progress.runid FROM progress INNER JOIN runs
            WHERE progress.runid=runs.id AND runs.plannerid=?""",
                (planner[0],),
            )
            runids = [t[0] for t in cur.fetchall()]
            timeTable = []
            dataTable = []
            for r in runids:
                # Select data for given run
                cur.execute(
                    "SELECT time, %s FROM progress WHERE runid = %s ORDER BY time"
                    % (attribute, r)
                )
                (time, data) = zip(*(cur.fetchall()))
                timeTable.append(time)
                dataTable.append(data)
            # It's conceivable that the sampling process may have
            # generated more samples for one run than another; in this
            # case, truncate all data series to length of shortest
            # one.
            fewestSamples = min(len(time[:]) for time in timeTable)
            times = np.array(timeTable[0][:fewestSamples])
            dataArrays = np.array([data[:fewestSamples] for data in dataTable])
            filteredData = ma.masked_array(
                dataArrays, np.equal(dataArrays, None), dtype=float
            )

            means = np.mean(filteredData, axis=0)
            stddevs = np.std(filteredData, axis=0, ddof=1)

            # plot average with error bars
            plt.errorbar(
                times, means, yerr=2 * stddevs, errorevery=max(1, len(times) // 20)
            )
            ax.legend(plannerNames)
    if len(plannerNames) > 0:
        plt.show()
    else:
        plt.clf()


def plotStatistics(dbname, fname):
    """Create a PDF file with box plots for all attributes."""
    print("Generating plots...")
    conn = sqlite3.connect(dbname)
    c = conn.cursor()
    c.execute("PRAGMA FOREIGN_KEYS = ON")
    c.execute("SELECT id, name FROM plannerConfigs")
    planners = [
        (t[0], t[1].replace("geometric_", "").replace("control_", ""))
        for t in c.fetchall()
    ]
    c.execute("PRAGMA table_info(runs)")
    colInfo = c.fetchall()[3:]

    pp = PdfPages(fname)
    for col in colInfo:
        if (
            col[2] == "BOOLEAN"
            or col[2] == "ENUM"
            or col[2] == "INTEGER"
            or col[2] == "REAL"
        ):
            plotAttribute(c, planners, col[1], col[2])
            pp.savefig(plt.gcf())

    c.execute("PRAGMA table_info(progress)")
    colInfo = c.fetchall()[2:]
    for col in colInfo:
        plotProgressAttribute(c, planners, col[1])
        pp.savefig(plt.gcf())
    plt.clf()

    pagey = 0.9
    pagex = 0.06
    c.execute("""SELECT id, name, timelimit, memorylimit FROM experiments""")
    experiments = c.fetchall()
    for experiment in experiments:
        c.execute(
            """SELECT count(*) FROM runs WHERE runs.experimentid = %d
            GROUP BY runs.plannerid"""
            % experiment[0]
        )
        numRuns = [run[0] for run in c.fetchall()]
        numRuns = numRuns[0] if len(set(numRuns)) == 1 else ",".join(numRuns)

        plt.figtext(pagex, pagey, 'Experiment "%s"' % experiment[1])
        plt.figtext(pagex, pagey - 0.05, "Number of averaged runs: %d" % numRuns)
        plt.figtext(
            pagex, pagey - 0.10, "Time limit per run: %g seconds" % experiment[2]
        )
        plt.figtext(pagex, pagey - 0.15, "Memory limit per run: %g MB" % experiment[3])
        pagey -= 0.22
    plt.show()
    pp.savefig(plt.gcf())
    pp.close()


def saveAsMysql(dbname, mysqldump):
    # See http://stackoverflow.com/questions/1067060/perl-to-python
    import re

    print("Saving as MySQL dump file...")

    conn = sqlite3.connect(dbname)
    mysqldump = open(mysqldump, "w")

    # make sure all tables are dropped in an order that keepd foreign keys valid
    c = conn.cursor()
    c.execute("SELECT name FROM sqlite_master WHERE type='table'")
    table_names = [str(t[0]) for t in c.fetchall()]
    c.close()
    last = ["experiments", "planner_configs"]
    for table in table_names:
        if table.startswith("sqlite"):
            continue
        if not table in last:
            mysqldump.write("DROP TABLE IF EXISTS `%s`;\n" % table)
    for table in last:
        if table in table_names:
            mysqldump.write("DROP TABLE IF EXISTS `%s`;\n" % table)

    for line in conn.iterdump():
        process = False
        for nope in (
            "BEGIN TRANSACTION",
            "COMMIT",
            "sqlite_sequence",
            "CREATE UNIQUE INDEX",
            "CREATE VIEW",
        ):
            if nope in line:
                break
        else:
            process = True
        if not process:
            continue
        line = re.sub(r"[\n\r\t ]+", " ", line)
        m = re.search("CREATE TABLE ([a-zA-Z0-9_]*)(.*)", line)
        if m:
            name, sub = m.groups()
            sub = sub.replace('"', "`")
            line = """CREATE TABLE IF NOT EXISTS %(name)s%(sub)s"""
            line = line % dict(name=name, sub=sub)
            # make sure we use an engine that supports foreign keys
            line = line.rstrip("\n\t ;") + " ENGINE = InnoDB;\n"
        else:
            m = re.search('INSERT INTO "([a-zA-Z0-9_]*)"(.*)', line)
            if m:
                line = "INSERT INTO %s%s\n" % m.groups()
                line = line.replace('"', r"\"")
                line = line.replace('"', "'")

        line = re.sub(r"([^'])'t'(.)", "\\1THIS_IS_TRUE\\2", line)
        line = line.replace("THIS_IS_TRUE", "1")
        line = re.sub(r"([^'])'f'(.)", "\\1THIS_IS_FALSE\\2", line)
        line = line.replace("THIS_IS_FALSE", "0")
        line = line.replace("AUTOINCREMENT", "AUTO_INCREMENT")
        mysqldump.write(line)
    mysqldump.close()


def computeViews(dbname):
    conn = sqlite3.connect(dbname)
    c = conn.cursor()
    c.execute("PRAGMA FOREIGN_KEYS = ON")
    c.execute("PRAGMA table_info(runs)")
    # kinodynamic paths cannot be simplified (or least not easily),
    # so simplification_time may not exist as a database column
    if "simplification_time" in [col[1] for col in c.fetchall()]:
        s0 = """SELECT plannerid, plannerConfigs.name AS plannerName, experimentid, solved, time + simplification_time AS total_time
            FROM plannerConfigs INNER JOIN experiments INNER JOIN runs
            ON plannerConfigs.id=runs.plannerid AND experiments.id=runs.experimentid"""
    else:
        s0 = """SELECT plannerid, plannerConfigs.name AS plannerName, experimentid, solved, time AS total_time
            FROM plannerConfigs INNER JOIN experiments INNER JOIN runs
            ON plannerConfigs.id=runs.plannerid AND experiments.id=runs.experimentid"""
    s1 = (
        """SELECT plannerid, plannerName, experimentid, AVG(solved) AS avg_solved, AVG(total_time) AS avg_total_time
        FROM (%s) GROUP BY plannerid, experimentid"""
        % s0
    )
    s2 = (
        """SELECT plannerid, experimentid, MIN(avg_solved) AS avg_solved, avg_total_time
        FROM (%s) GROUP BY plannerName, experimentid ORDER BY avg_solved DESC, avg_total_time ASC"""
        % s1
    )
    c.execute("DROP VIEW IF EXISTS bestPlannerConfigsPerExperiment")
    c.execute("CREATE VIEW IF NOT EXISTS bestPlannerConfigsPerExperiment AS %s" % s2)

    s1 = (
        """SELECT plannerid, plannerName, AVG(solved) AS avg_solved, AVG(total_time) AS avg_total_time
        FROM (%s) GROUP BY plannerid"""
        % s0
    )
    s2 = (
        """SELECT plannerid, MIN(avg_solved) AS avg_solved, avg_total_time
        FROM (%s) GROUP BY plannerName ORDER BY avg_solved DESC, avg_total_time ASC"""
        % s1
    )
    c.execute("DROP VIEW IF EXISTS bestPlannerConfigs")
    c.execute("CREATE VIEW IF NOT EXISTS bestPlannerConfigs AS %s" % s2)

    conn.commit()
    c.close()


if __name__ == "__main__":
    usage = """%prog [options] [<benchmark.log> ...]"""
    parser = OptionParser("A script to parse benchmarking results.\n" + usage)
    parser.add_option(
        "-d",
        "--database",
        dest="dbname",
        default="benchmark.db",
        help="Filename of benchmark database [default: %default]",
    )
    parser.add_option(
        "-v",
        "--view",
        action="store_true",
        dest="view",
        default=False,
        help="Compute the views for best planner configurations",
    )
    parser.add_option(
        "-p",
        "--plot",
        dest="plot",
        default=None,
        help="Create a PDF of plots with the filename provided",
    )
    parser.add_option(
        "-m",
        "--mysql",
        dest="mysqldb",
        default=None,
        help="Save SQLite3 database as a MySQL dump file",
    )
    (options, args) = parser.parse_args()

    if len(args) == 0:
        parser.error("No arguments were provided. Please provide full path of log file")

    if len(args) > 0:
        readBenchmarkLog(options.dbname, args)
        # If we update the database, we recompute the views as well
        options.view = True

    if options.view:
        computeViews(options.dbname)

    if options.plot:
        plotStatistics(options.dbname, options.plot)

    if options.mysqldb:
        saveAsMysql(options.dbname, options.mysqldb)
