#include "../src/Waveform.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum WaveformType {
    WAVEFORM_RASTER,
    WAVEFORM_CLOCK,
    WAVEFORM_PARK,
    WAVEFORM_UNPARK,
};

enum OutputFormat {
    FORMAT_AUTO,
    FORMAT_CSV,
    FORMAT_RAW,
};

struct Args {
    enum WaveformType type;
    enum OutputFormat format;
    const char *outputFile;

    uint32_t resolution;
    uint32_t width;
    uint32_t height;
    uint32_t xOffset;
    uint32_t yOffset;
    double zoom;
    uint32_t undershoot;
    double xformMatrix[4];
    double xformOffsetX;
    double xformOffsetY;
    int32_t xPark;
    int32_t yPark;
    double prevXParkVoltage;
    double prevYParkVoltage;

    int hasResolution;
    int hasWidth;
    int hasHeight;
};

static int ParseUint32(const char *str, const char *name, uint32_t *out) {
    char *end;
    unsigned long val = strtoul(str, &end, 10);
    if (*end != '\0' || end == str) {
        fprintf(stderr, "Error: invalid value '%s' for %s\n", str, name);
        return 0;
    }
    *out = (uint32_t)val;
    return 1;
}

static int ParseInt32(const char *str, const char *name, int32_t *out) {
    char *end;
    long val = strtol(str, &end, 10);
    if (*end != '\0' || end == str) {
        fprintf(stderr, "Error: invalid value '%s' for %s\n", str, name);
        return 0;
    }
    *out = (int32_t)val;
    return 1;
}

static int ParseDouble(const char *str, const char *name, double *out) {
    char *end;
    double val = strtod(str, &end);
    if (*end != '\0' || end == str) {
        fprintf(stderr, "Error: invalid value '%s' for %s\n", str, name);
        return 0;
    }
    *out = val;
    return 1;
}

static void PrintUsage(void) {
    fprintf(
        stderr,
        "Usage: WaveformTool <type> -o <file> [options]\n"
        "\n"
        "Types: raster, clock, park, unpark\n"
        "\n"
        "Options:\n"
        "  -o <file>                Output file (required)\n"
        "  --format csv|raw         Override format (auto from extension)\n"
        "  --resolution <n>         Scanner resolution\n"
        "  --width <n>              ROI width (default: resolution)\n"
        "  --height <n>             ROI height (default: resolution)\n"
        "  --xoffset <n>            ROI X offset (default: 0)\n"
        "  --yoffset <n>            ROI Y offset (default: 0)\n"
        "  --zoom <f>               Zoom factor (default: 1.0)\n"
        "  --undershoot <n>         Undershoot / line delay (default: 0)\n"
        "  --tform <a,b,c,d>        Affine 2x2 matrix, row-major\n"
        "  --tform-offset <tx,ty>   Affine translation in volts\n"
        "  --xpark <n>              X park position (default: 0)\n"
        "  --ypark <n>              Y park position (default: 0)\n"
        "  --prev-xpark-voltage <f> Previous X park voltage (default: 0)\n"
        "  --prev-ypark-voltage <f> Previous Y park voltage (default: 0)\n"
        "\n"
        "Required parameters:\n"
        "  raster:       --resolution\n"
        "  clock:        --width and --height (or --resolution)\n"
        "  park, unpark: --resolution\n"
        "\n"
        "Examples:\n"
        "  WaveformTool raster -o scan.csv --resolution 256\n"
        "  WaveformTool raster -o scan.raw --resolution 256 --undershoot 10\n"
        "  WaveformTool clock -o clock.csv --resolution 64 --undershoot 5\n"
        "  WaveformTool park -o park.csv --resolution 256\n");
}

static int ParseTform(const char *str, double m[4]) {
    char *buf = _strdup(str);
    if (!buf) {
        fprintf(stderr, "Error: out of memory\n");
        return 0;
    }
    char *tok = strtok(buf, ",");
    for (int i = 0; i < 4; i++) {
        if (!tok) {
            fprintf(stderr,
                    "Error: --tform requires 4 comma-separated values\n");
            free(buf);
            return 0;
        }
        char *end;
        m[i] = strtod(tok, &end);
        if (*end != '\0' || end == tok) {
            fprintf(stderr, "Error: invalid value '%s' in --tform\n", tok);
            free(buf);
            return 0;
        }
        tok = strtok(NULL, ",");
    }
    if (tok) {
        fprintf(stderr, "Error: --tform requires exactly 4 values\n");
        free(buf);
        return 0;
    }
    free(buf);
    return 1;
}

static int ParseTformOffset(const char *str, double *tx, double *ty) {
    char *buf = _strdup(str);
    if (!buf) {
        fprintf(stderr, "Error: out of memory\n");
        return 0;
    }
    char *tok = strtok(buf, ",");
    if (!tok) {
        fprintf(stderr,
                "Error: --tform-offset requires 2 comma-separated values\n");
        free(buf);
        return 0;
    }
    char *end;
    *tx = strtod(tok, &end);
    if (*end != '\0' || end == tok) {
        fprintf(stderr, "Error: invalid value '%s' in --tform-offset\n", tok);
        free(buf);
        return 0;
    }
    tok = strtok(NULL, ",");
    if (!tok) {
        fprintf(stderr,
                "Error: --tform-offset requires 2 comma-separated values\n");
        free(buf);
        return 0;
    }
    *ty = strtod(tok, &end);
    if (*end != '\0' || end == tok) {
        fprintf(stderr, "Error: invalid value '%s' in --tform-offset\n", tok);
        free(buf);
        return 0;
    }
    if (strtok(NULL, ",")) {
        fprintf(stderr, "Error: --tform-offset requires exactly 2 values\n");
        free(buf);
        return 0;
    }
    free(buf);
    return 1;
}

static int ParseArgs(int argc, char *argv[], struct Args *args) {
    memset(args, 0, sizeof(*args));
    args->zoom = 1.0;
    args->xformMatrix[0] = 1.0;
    args->xformMatrix[1] = 0.0;
    args->xformMatrix[2] = 0.0;
    args->xformMatrix[3] = 1.0;
    args->format = FORMAT_AUTO;

    if (argc < 2) {
        PrintUsage();
        return 0;
    }

    if (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0) {
        PrintUsage();
        return 0;
    }

    if (strcmp(argv[1], "raster") == 0)
        args->type = WAVEFORM_RASTER;
    else if (strcmp(argv[1], "clock") == 0)
        args->type = WAVEFORM_CLOCK;
    else if (strcmp(argv[1], "park") == 0)
        args->type = WAVEFORM_PARK;
    else if (strcmp(argv[1], "unpark") == 0)
        args->type = WAVEFORM_UNPARK;
    else {
        fprintf(stderr, "Error: unknown waveform type '%s'\n", argv[1]);
        return 0;
    }

    for (int i = 2; i < argc; i++) {
        if ((strcmp(argv[i], "-o") == 0) && i + 1 < argc) {
            args->outputFile = argv[++i];
        } else if (strcmp(argv[i], "--format") == 0 && i + 1 < argc) {
            i++;
            if (strcmp(argv[i], "csv") == 0)
                args->format = FORMAT_CSV;
            else if (strcmp(argv[i], "raw") == 0)
                args->format = FORMAT_RAW;
            else {
                fprintf(stderr, "Error: unknown format '%s'\n", argv[i]);
                return 0;
            }
        } else if (strcmp(argv[i], "--resolution") == 0 && i + 1 < argc) {
            if (!ParseUint32(argv[++i], "--resolution", &args->resolution))
                return 0;
            args->hasResolution = 1;
        } else if (strcmp(argv[i], "--width") == 0 && i + 1 < argc) {
            if (!ParseUint32(argv[++i], "--width", &args->width))
                return 0;
            args->hasWidth = 1;
        } else if (strcmp(argv[i], "--height") == 0 && i + 1 < argc) {
            if (!ParseUint32(argv[++i], "--height", &args->height))
                return 0;
            args->hasHeight = 1;
        } else if (strcmp(argv[i], "--xoffset") == 0 && i + 1 < argc) {
            if (!ParseUint32(argv[++i], "--xoffset", &args->xOffset))
                return 0;
        } else if (strcmp(argv[i], "--yoffset") == 0 && i + 1 < argc) {
            if (!ParseUint32(argv[++i], "--yoffset", &args->yOffset))
                return 0;
        } else if (strcmp(argv[i], "--zoom") == 0 && i + 1 < argc) {
            if (!ParseDouble(argv[++i], "--zoom", &args->zoom))
                return 0;
        } else if (strcmp(argv[i], "--undershoot") == 0 && i + 1 < argc) {
            if (!ParseUint32(argv[++i], "--undershoot", &args->undershoot))
                return 0;
        } else if (strcmp(argv[i], "--tform") == 0 && i + 1 < argc) {
            if (!ParseTform(argv[++i], args->xformMatrix))
                return 0;
        } else if (strcmp(argv[i], "--tform-offset") == 0 && i + 1 < argc) {
            if (!ParseTformOffset(argv[++i], &args->xformOffsetX,
                                  &args->xformOffsetY))
                return 0;
        } else if (strcmp(argv[i], "--xpark") == 0 && i + 1 < argc) {
            if (!ParseInt32(argv[++i], "--xpark", &args->xPark))
                return 0;
        } else if (strcmp(argv[i], "--ypark") == 0 && i + 1 < argc) {
            if (!ParseInt32(argv[++i], "--ypark", &args->yPark))
                return 0;
        } else if (strcmp(argv[i], "--prev-xpark-voltage") == 0 &&
                   i + 1 < argc) {
            if (!ParseDouble(argv[++i], "--prev-xpark-voltage",
                             &args->prevXParkVoltage))
                return 0;
        } else if (strcmp(argv[i], "--prev-ypark-voltage") == 0 &&
                   i + 1 < argc) {
            if (!ParseDouble(argv[++i], "--prev-ypark-voltage",
                             &args->prevYParkVoltage))
                return 0;
        } else {
            fprintf(stderr, "Error: unknown option '%s'\n", argv[i]);
            return 0;
        }
    }

    if (!args->outputFile) {
        fprintf(stderr, "Error: -o <file> is required\n");
        return 0;
    }

    switch (args->type) {
    case WAVEFORM_RASTER:
    case WAVEFORM_PARK:
    case WAVEFORM_UNPARK:
        if (!args->hasResolution) {
            fprintf(stderr, "Error: --resolution is required for %s\n",
                    argv[1]);
            return 0;
        }
        if (!args->hasWidth)
            args->width = args->resolution;
        if (!args->hasHeight)
            args->height = args->resolution;
        break;
    case WAVEFORM_CLOCK:
        if (!args->hasWidth && !args->hasHeight && args->hasResolution) {
            args->width = args->resolution;
            args->height = args->resolution;
            args->hasWidth = 1;
            args->hasHeight = 1;
        }
        if (!args->hasWidth || !args->hasHeight) {
            fprintf(stderr,
                    "Error: --width and --height are required for clock "
                    "(or use --resolution to default both)\n");
            return 0;
        }
        break;
    }

    return 1;
}

static int ResolveFormat(struct Args *args) {
    if (args->format != FORMAT_AUTO)
        return 1;

    const char *dot = strrchr(args->outputFile, '.');
    if (dot) {
        if (_stricmp(dot, ".csv") == 0) {
            args->format = FORMAT_CSV;
            return 1;
        }
        if (_stricmp(dot, ".raw") == 0) {
            args->format = FORMAT_RAW;
            return 1;
        }
    }

    fprintf(stderr,
            "Error: cannot determine format from extension of '%s'; "
            "use --format csv|raw\n",
            args->outputFile);
    return 0;
}

static void PopulateParams(const struct Args *args,
                           struct WaveformParams *params) {
    params->width = args->width;
    params->height = args->height;
    params->resolution = args->resolution;
    params->zoom = args->zoom;
    params->undershoot = args->undershoot;
    params->xOffset = args->xOffset;
    params->yOffset = args->yOffset;
    memcpy(params->xformMatrix, args->xformMatrix,
           sizeof(params->xformMatrix));
    params->xformOffsetX = args->xformOffsetX;
    params->xformOffsetY = args->xformOffsetY;
    params->xPark = args->xPark;
    params->yPark = args->yPark;
    params->prevXParkVoltage = args->prevXParkVoltage;
    params->prevYParkVoltage = args->prevYParkVoltage;
}

static int WriteXYCsv(FILE *f, const double *xy, uint32_t n) {
    if (fprintf(f, "x,y\n") < 0)
        return 0;
    for (uint32_t i = 0; i < n; i++) {
        if (fprintf(f, "%.17g,%.17g\n", xy[i], xy[i + n]) < 0)
            return 0;
    }
    return 1;
}

static int WriteXYRaw(FILE *f, const double *xy, uint32_t n) {
    size_t total = (size_t)n * 2;
    return fwrite(xy, sizeof(double), total, f) == total;
}

static int ExportRaster(const struct Args *args) {
    struct WaveformParams params;
    PopulateParams(args, &params);

    uint32_t samplesPerChan = (uint32_t)GetScannerWaveformSize(&params);
    uint32_t bufferSize = samplesPerChan * 2;
    double *xy = malloc(sizeof(double) * bufferSize);
    if (!xy) {
        fprintf(stderr, "Error: out of memory\n");
        return 0;
    }

    GenerateGalvoWaveformFrame(&params, xy);

    FILE *f = fopen(args->outputFile, args->format == FORMAT_CSV ? "w" : "wb");
    if (!f) {
        fprintf(stderr, "Error: cannot open '%s' for writing\n",
                args->outputFile);
        free(xy);
        return 0;
    }

    int ok;
    if (args->format == FORMAT_CSV)
        ok = WriteXYCsv(f, xy, samplesPerChan);
    else
        ok = WriteXYRaw(f, xy, samplesPerChan);

    fclose(f);
    free(xy);

    if (!ok) {
        fprintf(stderr, "Error: write failed\n");
        return 0;
    }

    printf("total sample count = %lu\n", (unsigned long)bufferSize);
    return 1;
}

static int ExportClock(const struct Args *args) {
    struct WaveformParams params;
    PopulateParams(args, &params);

    uint32_t n = (uint32_t)GetClockWaveformSize(&params);
    uint8_t *lineClock = malloc(n);
    uint8_t *lineClockFLIM = malloc(n);
    uint8_t *frameClockFLIM = malloc(n);
    if (!lineClock || !lineClockFLIM || !frameClockFLIM) {
        fprintf(stderr, "Error: out of memory\n");
        free(lineClock);
        free(lineClockFLIM);
        free(frameClockFLIM);
        return 0;
    }

    GenerateLineClock(&params, lineClock);
    GenerateFLIMLineClock(&params, lineClockFLIM);
    GenerateFLIMFrameClock(&params, frameClockFLIM);

    FILE *f = fopen(args->outputFile, args->format == FORMAT_CSV ? "w" : "wb");
    if (!f) {
        fprintf(stderr, "Error: cannot open '%s' for writing\n",
                args->outputFile);
        free(lineClock);
        free(lineClockFLIM);
        free(frameClockFLIM);
        return 0;
    }

    int ok = 1;
    if (args->format == FORMAT_CSV) {
        if (fprintf(f, "lineClock,lineClockFLIM,frameClockFLIM\n") < 0)
            ok = 0;
        for (uint32_t i = 0; i < n && ok; i++) {
            if (fprintf(f, "%u,%u,%u\n", lineClock[i], lineClockFLIM[i],
                        frameClockFLIM[i]) < 0)
                ok = 0;
        }
    } else {
        if (fwrite(lineClock, 1, n, f) != n)
            ok = 0;
        if (fwrite(lineClockFLIM, 1, n, f) != n)
            ok = 0;
        if (fwrite(frameClockFLIM, 1, n, f) != n)
            ok = 0;
    }

    fclose(f);
    free(lineClock);
    free(lineClockFLIM);
    free(frameClockFLIM);

    if (!ok) {
        fprintf(stderr, "Error: write failed\n");
        return 0;
    }

    printf("total sample count = %lu\n", (unsigned long)n * 3);
    return 1;
}

static int ExportPark(const struct Args *args) {
    struct WaveformParams params;
    PopulateParams(args, &params);

    uint32_t samplesPerChan = (uint32_t)GetParkWaveformSize(&params);
    uint32_t bufferSize = samplesPerChan * 2;
    double *xy = malloc(sizeof(double) * bufferSize);
    if (!xy) {
        fprintf(stderr, "Error: out of memory\n");
        return 0;
    }

    GenerateGalvoParkWaveform(&params, xy);

    FILE *f = fopen(args->outputFile, args->format == FORMAT_CSV ? "w" : "wb");
    if (!f) {
        fprintf(stderr, "Error: cannot open '%s' for writing\n",
                args->outputFile);
        free(xy);
        return 0;
    }

    int ok;
    if (args->format == FORMAT_CSV)
        ok = WriteXYCsv(f, xy, samplesPerChan);
    else
        ok = WriteXYRaw(f, xy, samplesPerChan);

    fclose(f);
    free(xy);

    if (!ok) {
        fprintf(stderr, "Error: write failed\n");
        return 0;
    }

    printf("total sample count = %lu\n", (unsigned long)bufferSize);
    return 1;
}

static int ExportUnpark(const struct Args *args) {
    struct WaveformParams params;
    PopulateParams(args, &params);

    uint32_t samplesPerChan = (uint32_t)GetParkWaveformSize(&params);
    uint32_t bufferSize = samplesPerChan * 2;
    double *xy = malloc(sizeof(double) * bufferSize);
    if (!xy) {
        fprintf(stderr, "Error: out of memory\n");
        return 0;
    }

    GenerateGalvoUnparkWaveform(&params, xy);

    FILE *f = fopen(args->outputFile, args->format == FORMAT_CSV ? "w" : "wb");
    if (!f) {
        fprintf(stderr, "Error: cannot open '%s' for writing\n",
                args->outputFile);
        free(xy);
        return 0;
    }

    int ok;
    if (args->format == FORMAT_CSV)
        ok = WriteXYCsv(f, xy, samplesPerChan);
    else
        ok = WriteXYRaw(f, xy, samplesPerChan);

    fclose(f);
    free(xy);

    if (!ok) {
        fprintf(stderr, "Error: write failed\n");
        return 0;
    }

    printf("total sample count = %lu\n", (unsigned long)bufferSize);
    return 1;
}

int main(int argc, char *argv[]) {
    struct Args args;
    if (!ParseArgs(argc, argv, &args))
        return 1;
    if (!ResolveFormat(&args))
        return 1;

    switch (args.type) {
    case WAVEFORM_RASTER:
        return ExportRaster(&args) ? 0 : 1;
    case WAVEFORM_CLOCK:
        return ExportClock(&args) ? 0 : 1;
    case WAVEFORM_PARK:
        return ExportPark(&args) ? 0 : 1;
    case WAVEFORM_UNPARK:
        return ExportUnpark(&args) ? 0 : 1;
    }
    return 1;
}
