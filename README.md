

## Compile

```
# make build folder, needs to be empty
latexmk -pdf -quiet -jobname=build/main main.tex

# to update pdf continously 
latexmk -pdf -quiet -pvc -view=none -jobname=build/main main.tex
```
