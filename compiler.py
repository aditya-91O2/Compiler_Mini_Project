"""
Robot Movement Language Compiler  ->  ESP32 (Arduino/C++) code
Pipeline: Lexer -> Parser (AST) -> Semantic checks -> Movement IR -> Code generator
No external libraries needed (pure Python 3).

Language:
    FORWARD n;   BACKWARD n;   LEFT deg;   RIGHT deg;
    REPEAT n { ... }          // comments start with //

Usage:
    python compiler.py examples/test1.rbt -o src/main.cpp              (stepper motors, Wokwi)
    python compiler.py examples/test1.rbt -o src/main.cpp --target dc  (real L298N car)
    python compiler.py examples/test1.rbt --tokens --ast --ir --trace
    python compiler.py examples/test1.rbt --send 192.168.4.1           (wireless, no USB)
"""
import argparse, math, sys
from dataclasses import dataclass

# ---------------- limits / config ----------------
ARENA = 100            # arena is ARENA x ARENA units, robot starts at the centre
MAX_MOVE = 100
MAX_REPEAT = 50
MAX_DEPTH = 3
KEYWORDS = {"FORWARD", "BACKWARD", "LEFT", "RIGHT", "REPEAT"}


class CompileError(Exception):
    def __init__(self, msg, line, col, phase):
        super().__init__(msg)
        self.msg, self.line, self.col, self.phase = msg, line, col, phase


# ---------------- 1. LEXER ----------------
@dataclass
class Token:
    kind: str      # KEYWORD, NUMBER, LBRACE, RBRACE, SEMI, EOF
    value: str
    line: int
    col: int


def lex(src):
    tokens, i, line, col = [], 0, 1, 1
    while i < len(src):
        c = src[i]
        if c == "\n":
            i += 1; line += 1; col = 1
        elif c in " \t\r":
            i += 1; col += 1
        elif src.startswith("//", i):
            while i < len(src) and src[i] != "\n":
                i += 1
        elif c.isdigit() or (c == "-" and i + 1 < len(src) and src[i + 1].isdigit()):
            j = i + 1
            while j < len(src) and src[j].isdigit():
                j += 1
            tokens.append(Token("NUMBER", src[i:j], line, col))
            col += j - i; i = j
        elif c.isalpha():
            j = i
            while j < len(src) and (src[j].isalnum() or src[j] == "_"):
                j += 1
            word = src[i:j].upper()
            if word not in KEYWORDS:
                raise CompileError(f"Unknown command '{src[i:j]}'", line, col, "lexer")
            tokens.append(Token("KEYWORD", word, line, col))
            col += j - i; i = j
        elif c == "{":
            tokens.append(Token("LBRACE", c, line, col)); i += 1; col += 1
        elif c == "}":
            tokens.append(Token("RBRACE", c, line, col)); i += 1; col += 1
        elif c == ";":
            tokens.append(Token("SEMI", c, line, col)); i += 1; col += 1
        else:
            raise CompileError(f"Illegal character '{c}'", line, col, "lexer")
    tokens.append(Token("EOF", "", line, col))
    return tokens


# ---------------- 2. PARSER (builds AST) ----------------
@dataclass
class Move:
    op: str        # FORWARD / BACKWARD / LEFT / RIGHT
    value: int
    line: int
    col: int


@dataclass
class Repeat:
    count: int
    body: list
    line: int
    col: int


class Parser:
    def __init__(self, tokens):
        self.t, self.p = tokens, 0

    def peek(self):
        return self.t[self.p]

    def eat(self, kind, what):
        tok = self.peek()
        if tok.kind != kind:
            got = "end of file" if tok.kind == "EOF" else f"'{tok.value}'"
            raise CompileError(f"Expected {what} but found {got}", tok.line, tok.col, "parser")
        self.p += 1
        return tok

    def program(self):
        stmts = []
        while self.peek().kind != "EOF":
            stmts.append(self.statement())
        return stmts

    def statement(self):
        tok = self.eat("KEYWORD", "a command")
        if tok.value == "REPEAT":
            n = self.eat("NUMBER", "a repeat count")
            self.eat("LBRACE", "'{'")
            body = []
            while self.peek().kind not in ("RBRACE", "EOF"):
                body.append(self.statement())
            if self.peek().kind == "EOF":
                raise CompileError("Missing '}' - REPEAT block is never closed",
                                   tok.line, tok.col, "parser")
            self.eat("RBRACE", "'}'")
            return Repeat(int(n.value), body, tok.line, tok.col)
        n = self.eat("NUMBER", f"a number after {tok.value}")
        self.eat("SEMI", "';'")
        return Move(tok.value, int(n.value), tok.line, tok.col)


# ---------------- 3. SEMANTIC CHECKS + 4. MOVEMENT IR ----------------
class Analyzer:
    """Validates values/nesting and simulates the path to check arena boundaries.
    Produces a flat (loop-expanded) IR list of (op, value)."""
    def __init__(self):
        self.x = self.y = ARENA / 2
        self.heading = 90.0      # degrees, 90 = facing 'up' on the map
        self.ir = []

    def check(self, stmts, depth=0):
        for s in stmts:
            if isinstance(s, Repeat):
                if depth + 1 > MAX_DEPTH:
                    raise CompileError(f"REPEAT nested deeper than {MAX_DEPTH}", s.line, s.col, "semantic")
                if not 1 <= s.count <= MAX_REPEAT:
                    raise CompileError(f"REPEAT count must be 1..{MAX_REPEAT}, got {s.count}", s.line, s.col, "semantic")
                if not s.body:
                    raise CompileError("REPEAT block is empty", s.line, s.col, "semantic")
                for _ in range(s.count):
                    self.check(s.body, depth + 1)
            else:
                self.move(s)

    def move(self, s):
        if s.op in ("FORWARD", "BACKWARD"):
            if not 1 <= s.value <= MAX_MOVE:
                raise CompileError(f"{s.op} distance must be 1..{MAX_MOVE}, got {s.value}", s.line, s.col, "semantic")
            d = s.value if s.op == "FORWARD" else -s.value
            self.x += d * math.cos(math.radians(self.heading))
            self.y += d * math.sin(math.radians(self.heading))
            if not (0 <= self.x <= ARENA and 0 <= self.y <= ARENA):
                raise CompileError(
                    f"Robot leaves the {ARENA}x{ARENA} arena at ({self.x:.0f},{self.y:.0f})",
                    s.line, s.col, "semantic")
        else:
            if not 1 <= s.value <= 360:
                raise CompileError(f"{s.op} angle must be 1..360, got {s.value}", s.line, s.col, "semantic")
            self.heading += s.value if s.op == "LEFT" else -s.value
        self.ir.append((s.op, s.value))


# ---------------- 5. CODE GENERATOR (ESP32 / Arduino C++) ----------------
CPP_DC = r'''// AUTO-GENERATED by Robot Movement Language Compiler - do not edit
#include <Arduino.h>

// ESP32 GPIO -> L298N input pins
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14

const int MS_PER_UNIT = 500;   // calibration: time per 1 distance unit
const float MS_PER_DEG = 3.04; // calibration: ms per degree of turn (tuned on the car: 2.5 gave ~74 deg for 90)

void setMotors(int a, int b, int c, int d) {
  digitalWrite(IN1, a); digitalWrite(IN2, b);
  digitalWrite(IN3, c); digitalWrite(IN4, d);
}
void stopMotors() { setMotors(0, 0, 0, 0); }

void forward(int n)  { Serial.printf("FORWARD %d\n", n);  setMotors(1,0,1,0); delay(n * MS_PER_UNIT); stopMotors(); delay(200); }
void backward(int n) { Serial.printf("BACKWARD %d\n", n); setMotors(0,1,0,1); delay(n * MS_PER_UNIT); stopMotors(); delay(200); }
void left(int d)     { Serial.printf("LEFT %d\n", d);     setMotors(1,0,0,1); delay((unsigned long)(d * MS_PER_DEG));  stopMotors(); delay(200); }
void right(int d)    { Serial.printf("RIGHT %d\n", d);    setMotors(0,1,1,0); delay((unsigned long)(d * MS_PER_DEG));  stopMotors(); delay(200); }

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();
  delay(500);
  Serial.println("START");

  // ---------- generated program ----------
@@BODY@@
  // ---------------------------------------
  Serial.println("DONE");
}

void loop() {}
'''


# Simulation target: two stepper motors (left wheel / right wheel) via A4988 drivers.
# Wokwi animates steppers, so the movements are visible on screen.
CPP_STEPPER = r'''// AUTO-GENERATED by Robot Movement Language Compiler - do not edit
// Target: ESP32 + 2 x A4988 + 2 x stepper motor (left wheel, right wheel)
#include <Arduino.h>

#define L_STEP 26
#define L_DIR  27
#define R_STEP 14
#define R_DIR  12

const int STEPS_PER_UNIT = 200;   // 1 distance unit = 1 full wheel revolution
const int STEPS_PER_DEG  = 2;     // 90 deg turn = 180 steps
const int STEP_US        = 1000;  // half-period of a step pulse (smaller = faster)

// run both wheels; dirL/dirR: HIGH or LOW
void drive(int steps, int dirL, int dirR) {
  digitalWrite(L_DIR, dirL);
  digitalWrite(R_DIR, dirR);
  for (int i = 0; i < steps; i++) {
    digitalWrite(L_STEP, HIGH); digitalWrite(R_STEP, HIGH);
    delayMicroseconds(STEP_US);
    digitalWrite(L_STEP, LOW);  digitalWrite(R_STEP, LOW);
    delayMicroseconds(STEP_US);
  }
  delay(300);   // short pause between commands
}

void forward(int n)  { Serial.printf("FORWARD %d\n", n);  drive(n * STEPS_PER_UNIT, HIGH, HIGH); }
void backward(int n) { Serial.printf("BACKWARD %d\n", n); drive(n * STEPS_PER_UNIT, LOW,  LOW);  }
void left(int d)     { Serial.printf("LEFT %d\n", d);     drive(d * STEPS_PER_DEG,  LOW,  HIGH); }
void right(int d)    { Serial.printf("RIGHT %d\n", d);    drive(d * STEPS_PER_DEG,  HIGH, LOW);  }

void setup() {
  Serial.begin(115200);
  pinMode(L_STEP, OUTPUT); pinMode(L_DIR, OUTPUT);
  pinMode(R_STEP, OUTPUT); pinMode(R_DIR, OUTPUT);
  delay(500);
  Serial.println("START");

  // ---------- generated program ----------
@@BODY@@
  // ---------------------------------------
  Serial.println("DONE");
}

void loop() {}
'''
TEMPLATES = {"stepper": CPP_STEPPER, "dc": CPP_DC}


def gen_body(stmts, indent=1, counter=None):
    counter = counter if counter is not None else [0]
    pad, out = "  " * indent, []
    for s in stmts:
        if isinstance(s, Repeat):
            v = "ijk"[counter[0] % 3] + str(counter[0]); counter[0] += 1
            out.append(f"{pad}for (int {v} = 0; {v} < {s.count}; {v}++) {{")
            out.extend(gen_body(s.body, indent + 1, counter))
            out.append(f"{pad}}}")
        else:
            out.append(f"{pad}{s.op.lower()}({s.value});")
    return out


# ---------------- helpers: printing ----------------
def show_ast(stmts, depth=0):
    for s in stmts:
        pad = "  " * depth
        if isinstance(s, Repeat):
            print(f"{pad}Repeat({s.count})")
            show_ast(s.body, depth + 1)
        else:
            print(f"{pad}{s.op.capitalize()}({s.value})")


def compile_source(src):
    tokens = lex(src)
    ast = Parser(tokens).program()
    an = Analyzer()
    an.check(ast)
    return tokens, ast, an.ir, an


# ---------------- 6. WIRELESS SENDER (WiFi) ----------------
MS_PER_UNIT_EST, MS_PER_DEG_EST = 500, 3.04      # only used to estimate run time
MAX_SEND = 1000                                   # must match MAX_CMDS in the firmware


def send_program(ir, target):
    """Send the movement list to the car over WiFi (TCP). Car must run firmware/wifi_car.cpp."""
    import socket
    host, _, port = target.partition(":")
    port = int(port) if port else 5000
    if len(ir) > MAX_SEND:
        print(f"error: program expands to {len(ir)} commands; the car holds at most {MAX_SEND}.", file=sys.stderr)
        sys.exit(1)
    try:
        sock = socket.create_connection((host, port), timeout=6)
    except OSError as e:
        print(f"error: cannot reach the car at {host}:{port} ({e}).\n"
              "Join the 'RobotCar' WiFi network first and check the car is powered.", file=sys.stderr)
        sys.exit(1)
    est = sum((v * MS_PER_UNIT_EST if op in ("FORWARD", "BACKWARD") else v * MS_PER_DEG_EST) + 200
              for op, v in ir) / 1000.0
    f = sock.makefile("r", newline="\n")
    try:
        if f.readline().strip() != "READY":
            print("error: car did not answer READY", file=sys.stderr); sys.exit(1)
        sock.sendall(("".join(f"{op} {v}\n" for op, v in ir) + "END\n").encode())
        print(f"Sent {len(ir)} commands (about {est:.0f} s to run). Ctrl+C = emergency stop.")
        sock.settimeout(est * 1.5 + 15)
        while True:
            line = f.readline().strip()
            if not line:
                print("Connection closed by car."); break
            print("car:", line)
            if line in ("DONE", "STOPPED") or line.startswith("ERR"):
                break
    except KeyboardInterrupt:
        try: sock.sendall(b"STOP\n")
        except OSError: pass
        print("\nSTOP sent.")
    except socket.timeout:
        print("error: timed out waiting for the car", file=sys.stderr)
    finally:
        sock.close()


def main():
    ap = argparse.ArgumentParser(description="Robot Movement Language -> ESP32 compiler")
    ap.add_argument("source")
    ap.add_argument("-o", "--output", help="output .cpp file")
    ap.add_argument("--tokens", action="store_true")
    ap.add_argument("--ast", action="store_true")
    ap.add_argument("--ir", action="store_true")
    ap.add_argument("--trace", action="store_true", help="run reference interpreter")
    ap.add_argument("--send", metavar="HOST[:PORT]",
                    help="send the compiled program to the car over WiFi, e.g. 192.168.4.1")
    ap.add_argument("--target", choices=["stepper", "dc"], default="stepper",
                    help="stepper = Wokwi simulation (default), dc = real L298N DC-motor car")
    a = ap.parse_args()

    with open(a.source, encoding="utf-8") as f:
        src = f.read()
    try:
        tokens, ast, ir, an = compile_source(src)
    except CompileError as e:
        print(f"{a.source}:{e.line}:{e.col}: {e.phase} error: {e.msg}", file=sys.stderr)
        lines = src.splitlines()
        if 0 < e.line <= len(lines):
            print("   " + lines[e.line - 1], file=sys.stderr)
            print("   " + " " * (e.col - 1) + "^", file=sys.stderr)
        sys.exit(1)

    if a.tokens:
        print("== TOKENS ==")
        for t in tokens[:-1]:
            print(f"{t.line}:{t.col}\t{t.kind}\t{t.value}")
    if a.ast:
        print("== AST ==");  show_ast(ast)
    if a.ir:
        print("== IR (loops expanded) ==")
        for i, (op, v) in enumerate(ir):
            print(f"{i:03d}  {op} {v}")
    if a.trace:
        print("== REFERENCE INTERPRETER TRACE ==")
        for op, v in ir:
            print(f"{op} {v}")
        print(f"Final position: ({an.x:.1f}, {an.y:.1f}), heading {an.heading % 360:.0f} deg")
    if a.output:
        code = TEMPLATES[a.target].replace("@@BODY@@", "\n".join(gen_body(ast)))
        with open(a.output, "w", encoding="utf-8", newline="\n") as f:
            f.write(code)
        print(f"OK: wrote {a.output}  ({len(ir)} movement commands, target={a.target})")
    if a.send:
        send_program(ir, a.send)


if __name__ == "__main__":
    main()
