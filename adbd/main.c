#include <stdio.h>
#include <stdlib.h>

#include <cutils/properties.h>

int main(int argc, char *argv[])
{
	int ret;
	char value[PROPERTY_VALUE_MAX];
	const char *default_value = "tutu";

	ret = property_get("", value, NULL);
	printf("[%d] ret %d, value '%s'\n", __LINE__, ret, value);
	ret = property_get("mode", value, NULL);
	printf("[%d] ret %d, value '%s'\n", __LINE__, ret, value);
	ret = property_get("plop", value, NULL);
	printf("[%d] ret %d, value '%s'\n", __LINE__, ret, value);


	ret = property_get("", value, default_value);
	printf("[%d] ret %d, value '%s'\n", __LINE__, ret, value);
	ret = property_get("mode", value, default_value);
	printf("[%d] ret %d, value '%s'\n", __LINE__, ret, value);
	ret = property_get("plop", value, default_value);
	printf("[%d] ret %d, value '%s'\n", __LINE__, ret, value);

	ret = property_get("foo", value, default_value);
	printf("[%d] ret %d, value '%s'\n", __LINE__, ret, value);

	return EXIT_SUCCESS;
}
