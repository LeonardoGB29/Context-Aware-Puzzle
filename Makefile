# Makefile — rompecabezas_grafica
# make        compila generator y solver
# make run    compila y corre el pipeline (generator -> solver)
# make clean  borra binarios, data/pieces/ y las imagenes generadas

CXX      := c++
CXXFLAGS := -std=c++17 -O2 -Wall $(shell pkg-config --cflags opencv4)
LDLIBS   := $(shell pkg-config --libs opencv4)

BIN := build
SRC := src
HEADERS := $(wildcard $(SRC)/*.hpp)

# modulos comunes que usa el solver
COMMON     := geometria gabor pieza ensamblado salida
COMMON_OBJ := $(addprefix $(BIN)/, $(addsuffix .o, $(COMMON)))

.PHONY: all run clean

all: $(BIN)/generator $(BIN)/solver

# cada .cpp -> .o (se recompila si cambia cualquier header)
$(BIN)/%.o: $(SRC)/%.cpp $(HEADERS) | $(BIN)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# generator solo necesita la geometria
$(BIN)/generator: $(BIN)/generator.o $(BIN)/geometria.o
	$(CXX) $^ -o $@ $(LDLIBS)

# solver necesita todos los modulos
$(BIN)/solver: $(BIN)/solver.o $(COMMON_OBJ)
	$(CXX) $^ -o $@ $(LDLIBS)

$(BIN):
	@mkdir -p $(BIN)

run: all
	./$(BIN)/generator
	./$(BIN)/solver

clean:
	rm -rf $(BIN)
	rm -rf data/pieces
	rm -f data/cut_preview.png data/resultado.png data/clasificacion.png
	@echo "Limpiado: build/, data/pieces/ (1_piezas_cortadas, 2_piezas_rotadas, 3_analisis_coloreo), previews"
