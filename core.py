
with open("out.txt", "r") as f:
  data = f.read()

lines = data.split("\n")
traces = [x for x in lines if x.startswith("T:")]

print(traces)

for trace in traces:
  d = trace.split(":")[1]
  ds = d.split(";")

  for i in range(len(ds)):
    