import plotly.express as px
# df = px.data.iris()
# fig = px.scatter_3d(df, x='sepal_length', y='sepal_width', z='petal_width',
#               color='species')
# fig.show()
df = px.data.gapminder().query("country=='Brazil'")
fig = px.line_3d(df, x="gdpPercap", y="pop", z="year")
fig.show()
