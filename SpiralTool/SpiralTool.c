#include "../src/Waveform.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum OutputFormat {
    FORMAT_AUTO,
    FORMAT_CSV,
    FORMAT_RAW,
};

struct Args {
    enum OutputFormat format;
    const char *outputFile;

    uint32_t resolution;
    uint32_t width;
    uint32_t height;
    uint32_t xOffset;
    uint32_t yOffset;
    double zoom;
    double turnSpacing;
    double turnDurationMs;
    double minRadius;
    double xCenterOffset;
    uint32_t numCycles;
    double xformMatrix[4];
    double xformOffsetX;
    double xformOffsetY;

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

static void PrintUsage(void) {
    fprintf(
        stderr,
        "Usage: SpiralTool -o <file> [options]\n"
        "\n"
        "Options:\n"
        "  -o <file>                Output file (required)\n"
        "  --format csv|raw         Override format (auto from extension)\n"
        "  --resolution <n>         Scanner resolution (required)\n"
        "  --width <n>              ROI width (default: resolution)\n"
        "  --height <n>             ROI height (default: resolution)\n"
        "  --xoffset <n>            ROI X offset (default: 0)\n"
        "  --yoffset <n>            ROI Y offset (default: 0)\n"
        "  --zoom <f>               Zoom factor (default: 1.0)\n"
        "  --turn-spacing <f>       Turn spacing in pixels (default: 10.0)\n"
        "  --turn-duration <f>      Turn duration in ms (default: 5.0)\n"
        "  --min-radius <f>         Minimum radius in pixels (default: 1.0)\n"
        "  --x-center-offset <f>    X center offset in pixels (default: 0.0)\n"
        "  --num-cycles <n>         Number of arm cycles (default: 10)\n"
        "  --tform <a,b,c,d>        Affine 2x2 matrix, row-major\n"
        "  --tform-offset <tx,ty>   Affine translation in volts\n"
        "\n"
        "Examples:\n"
        "  SpiralTool -o spiral.csv --resolution 256\n"
        "  SpiralTool -o spiral.raw --resolution 256 --turn-spacing 15\n");
}

static int ParseArgs(int argc, char *argv[], struct Args *args) {
    memset(args, 0, sizeof(*args));
    args->zoom = 1.0;
    args->turnSpacing = 10.0;
    args->turnDurationMs = 5.0;
    args->minRadius = 1.0;
    args->numCycles = 10;
    args->xformMatrix[0] = 1.0;
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

    for (int i = 1; i < argc; i++) {
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
        } else if (strcmp(argv[i], "--turn-spacing") == 0 && i + 1 < argc) {
            if (!ParseDouble(argv[++i], "--turn-spacing", &args->turnSpacing))
                return 0;
        } else if (strcmp(argv[i], "--turn-duration") == 0 && i + 1 < argc) {
            if (!ParseDouble(argv[++i], "--turn-duration",
                             &args->turnDurationMs))
                return 0;
        } else if (strcmp(argv[i], "--min-radius") == 0 && i + 1 < argc) {
            if (!ParseDouble(argv[++i], "--min-radius", &args->minRadius))
                return 0;
        } else if (strcmp(argv[i], "--x-center-offset") == 0 && i + 1 < argc) {
            if (!ParseDouble(argv[++i], "--x-center-offset",
                             &args->xCenterOffset))
                return 0;
        } else if (strcmp(argv[i], "--num-cycles") == 0 && i + 1 < argc) {
            if (!ParseUint32(argv[++i], "--num-cycles", &args->numCycles))
                return 0;
        } else if (strcmp(argv[i], "--tform") == 0 && i + 1 < argc) {
            if (!ParseTform(argv[++i], args->xformMatrix))
                return 0;
        } else if (strcmp(argv[i], "--tform-offset") == 0 && i + 1 < argc) {
            if (!ParseTformOffset(argv[++i], &args->xformOffsetX,
                                  &args->xformOffsetY))
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
    if (!args->hasResolution) {
        fprintf(stderr, "Error: --resolution is required\n");
        return 0;
    }
    if (!args->hasWidth)
        args->width = args->resolution;
    if (!args->hasHeight)
        args->height = args->resolution;

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

static void PopulateSpiralParams(const struct Args *args,
                                 struct SpiralWaveformParams *params) {
    double zoom = args->zoom;
    double resolution = (double)args->resolution;
    double width = (double)args->width;
    double height = (double)args->height;
    double xOffset = (double)args->xOffset;
    double yOffset = (double)args->yOffset;

    double minDim = (width < height) ? width : height;
    params->radius = minDim / (2.0 * zoom * resolution);
    params->centerX =
        (-0.5 * resolution + xOffset + width / 2.0 + args->xCenterOffset) /
        (zoom * resolution);
    params->centerY =
        (-0.5 * resolution + yOffset + height / 2.0) / (zoom * resolution);
    params->turnSpacing = args->turnSpacing / (zoom * resolution);
    params->turnDurationMs = args->turnDurationMs;
    params->rMin = args->minRadius / (zoom * resolution);
    for (int i = 0; i < 4; ++i)
        params->xformMatrix[i] = args->xformMatrix[i];
    params->xformOffsetX = args->xformOffsetX;
    params->xformOffsetY = args->xformOffsetY;
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

int main(int argc, char *argv[]) {
    struct Args args;
    if (!ParseArgs(argc, argv, &args))
        return 1;
    if (!ResolveFormat(&args))
        return 1;

    struct SpiralWaveformParams params;
    PopulateSpiralParams(&args, &params);

    struct SpiralGenState *state = CreateSpiralGenState(&params);
    if (!state) {
        fprintf(stderr, "Error: failed to create spiral state\n");
        return 1;
    }

    int32_t samplesPerCycle = GetSpiralArmCycleSamples(state);
    int32_t samplesPerArm = GetSpiralSamplesPerArm(state);
    int32_t centerConnSamples = GetSpiralCenterConnSamples(state);
    int32_t totalSamples = samplesPerCycle * (int32_t)args.numCycles;

    double *xy = malloc(sizeof(double) * (size_t)totalSamples * 2);
    if (!xy) {
        fprintf(stderr, "Error: out of memory\n");
        DestroySpiralGenState(state);
        return 1;
    }

    GenerateSpiralChunk(state, xy, totalSamples);
    DestroySpiralGenState(state);

    FILE *f = fopen(args.outputFile, args.format == FORMAT_CSV ? "w" : "wb");
    if (!f) {
        fprintf(stderr, "Error: cannot open '%s' for writing\n",
                args.outputFile);
        free(xy);
        return 1;
    }

    int ok;
    if (args.format == FORMAT_CSV)
        ok = WriteXYCsv(f, xy, (uint32_t)totalSamples);
    else
        ok = WriteXYRaw(f, xy, (uint32_t)totalSamples);

    fclose(f);
    free(xy);

    if (!ok) {
        fprintf(stderr, "Error: write failed\n");
        return 1;
    }

    printf("samples_per_cycle=%d\n", (int)samplesPerCycle);
    printf("samples_per_arm=%d\n", (int)samplesPerArm);
    printf("center_conn_samples=%d\n", (int)centerConnSamples);
    printf("total_samples=%d\n", (int)totalSamples);

    return 0;
}
